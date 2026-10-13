#!/usr/bin/env python3
"""ROS 2 interface to an Inspire dexterous hand over Modbus TCP (port 6000).

Finger order everywhere: little, ring, middle, index, thumb_bend, thumb_rotation.
Angles are the hand's raw units, 0..1000 (1000 = open/extended, 0 = closed/bent).

Interface:
    ~/state    sensor_msgs/JointState   position = angle / 1000, effort = measured force (raw)
    ~/command  sensor_msgs/JointState   position = target angle / 1000, either all six in order
                                        (names empty) or by name; unnamed fingers keep their target
                                        (or, before the first command, their measured angle)
    ~/open     std_srvs/Trigger         go to open_pose
    ~/close    std_srvs/Trigger         go to close_pose; fingers stop early at force_limit

The hand doesn't move until a command or service call arrives.
"""

import socket
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

FINGERS = ["little", "ring", "middle", "index", "thumb_bend", "thumb_rotation"]

# Holding registers. The hand is byte addressed: a register reads the two bytes at addr, addr + 1,
# so the 6 x int16 blocks span 6 registers and the 6 x uint8 blocks span 3.
ANGLE_SET = 1486
FORCE_SET = 1498
SPEED_SET = 1522
ANGLE_ACT = 1546
FORCE_ACT = 1582
ERROR = 1606  # 6 x uint8, then STATUS (1612) and TEMP (1618), read together


class ModbusTcp:
    def __init__(self, host, port, timeout):
        self.address = (host, port)
        self.timeout = timeout
        self.sock = None
        self.transaction = 0

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def _request(self, function, body):
        if self.sock is None:
            self.sock = socket.create_connection(self.address, timeout=self.timeout)
        self.transaction = (self.transaction + 1) & 0xFFFF
        pdu = struct.pack(">B", function) + body
        self.sock.sendall(struct.pack(">HHHB", self.transaction, 0, len(pdu) + 1, 1) + pdu)
        header = self._recv(7)
        transaction, _, length, _ = struct.unpack(">HHHB", header)
        response = self._recv(length - 1)
        if transaction != self.transaction:
            raise IOError(f"transaction {transaction}, expected {self.transaction}")
        if response[0] != function:
            raise IOError(f"Modbus exception {response[1] if len(response) > 1 else '?'}")
        return response[1:]

    def _recv(self, n):
        data = b""
        while len(data) < n:
            chunk = self.sock.recv(n - len(data))
            if not chunk:
                raise IOError("connection closed")
            data += chunk
        return data

    def read(self, address, count):
        """Read holding registers as signed 16-bit values."""
        response = self._request(3, struct.pack(">HH", address, count))
        return list(struct.unpack(f">{count}h", response[1 : 1 + 2 * count]))

    def read_bytes(self, address, count):
        """Read a block of uint8 fields (count bytes, count / 2 registers)."""
        words = self.read(address, (count + 1) // 2)
        return [b for w in words for b in (w & 0xFF, (w >> 8) & 0xFF)][:count]

    def write(self, address, values):
        values = [int(v) for v in values]
        body = struct.pack(f">HHB{len(values)}h", address, len(values), 2 * len(values), *values)
        self._request(16, body)


def six(node, name, default):
    value = list(node.declare_parameter(name, default).value)
    if len(value) != 6:
        raise ValueError(f"{name} needs 6 values ({', '.join(FINGERS)})")
    return value


class InspireHand(Node):
    def __init__(self):
        super().__init__("inspire_hand")
        host = self.declare_parameter("host", "192.168.123.211").value
        port = self.declare_parameter("port", 6000).value
        prefix = self.declare_parameter("joint_prefix", "right_").value
        rate = self.declare_parameter("rate", 20.0).value
        self.speed = six(self, "speed", [500] * 6)
        self.force_limit = six(self, "force_limit", [1000] * 6)
        self.open_pose = six(self, "open_pose", [1000, 1000, 1000, 1000, 1000, 0])
        self.close_pose = six(self, "close_pose", [0, 0, 0, 0, 0, 0])

        self.names = [prefix + f for f in FINGERS]
        self.hand = ModbusTcp(host, port, timeout=0.2)
        self.configured = False
        self.target = None  # last commanded angles; None until the first command
        self.measured = None

        self.state_pub = self.create_publisher(JointState, "~/state", 10)
        self.create_subscription(JointState, "~/command", self.on_command, 10)
        self.create_service(Trigger, "~/open", lambda req, res: self.go(self.open_pose, res, "open"))
        self.create_service(Trigger, "~/close", lambda req, res: self.go(self.close_pose, res, "close"))
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"Inspire hand at {host}:{port}")

    def configure(self):
        self.hand.write(SPEED_SET, self.speed)
        self.hand.write(FORCE_SET, self.force_limit)
        self.configured = True

    def send(self, angles):
        angles = [min(1000, max(0, int(round(a)))) for a in angles]
        if not self.configured:
            self.configure()
        self.hand.write(ANGLE_SET, angles)
        self.target = angles

    def go(self, pose, response, name):
        try:
            self.send(pose)
            response.success = True
            response.message = f"{name}: {pose}"
        except (OSError, IOError) as e:
            self.on_error(e)
            response.success = False
            response.message = str(e)
        return response

    def on_command(self, msg):
        if msg.name:
            angles = self.target if self.target is not None else self.measured
            if angles is None:
                self.get_logger().warn("no hand state yet", throttle_duration_sec=2.0)
                return
            angles = list(angles)
            for name, position in zip(msg.name, msg.position):
                if name not in self.names:
                    self.get_logger().warn(f"unknown finger {name}", throttle_duration_sec=2.0)
                    return
                angles[self.names.index(name)] = 1000.0 * position
        elif len(msg.position) == 6:
            angles = [1000.0 * p for p in msg.position]
        else:
            self.get_logger().warn("command needs 6 positions or names", throttle_duration_sec=2.0)
            return
        try:
            self.send(angles)
        except (OSError, IOError) as e:
            self.on_error(e)

    def tick(self):
        try:
            angles = self.hand.read(ANGLE_ACT, 6)
            forces = self.hand.read(FORCE_ACT, 6)
            error_status_temp = self.hand.read_bytes(ERROR, 18)
        except (OSError, IOError) as e:
            self.on_error(e)
            return
        self.measured = angles
        errors = error_status_temp[:6]
        if any(errors):
            self.get_logger().error(
                f"finger errors {dict(zip(FINGERS, errors))}, temperatures {error_status_temp[12:]}",
                throttle_duration_sec=2.0,
            )
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = [a / 1000.0 for a in angles]
        msg.effort = [float(f) for f in forces]
        self.state_pub.publish(msg)

    def on_error(self, e):
        self.get_logger().error(f"hand: {e}", throttle_duration_sec=2.0)
        self.hand.close()  # reconnect on the next request
        self.configured = False


def main():
    rclpy.init()
    node = InspireHand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.hand.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
