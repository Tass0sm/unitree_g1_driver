#!/usr/bin/env python3
"""Walk the G1 with Unitree's own walking controller, from body-frame velocity commands.

Sends the loco service's SetVelocity (what unitree_sdk2's LocoClient::Move does) as
unitree_api/Request on /api/sport/request, so it runs alongside the arm driver: Unitree's
controller keeps the legs, rt/arm_sdk the arms and waist.

The robot must be in a state that takes SDK velocity: 500 on older firmware, 501 on 1.5.x. On
1.5.x the remote's walk/run mode (R2+A) is FSM 802, which answers SetVelocity with success and
ignores it; 501 is reached with damp (1) -> stand up (4) -> 501 (unitree_sdk2_python issue 170,
xr_teleoperate PR 312). A state with no locomotion at all refuses commands with code 7301.

Interface:
    cmd_vel_topic  geometry_msgs/Twist   linear.x forward, linear.y left (m/s), angular.z
                                         counterclockwise (rad/s), all in the body frame
    ~/command      geometry_msgs/TwistStamped   what is actually sent, after the limits
    ~/stop         std_srvs/Trigger      stop now, and ignore cmd_vel until ~/resume
    ~/resume       std_srvs/Trigger      follow cmd_vel again
    ~/enter_walk   std_srvs/Trigger      put the robot in walk_fsm: damp (1) -> stand up (4) ->
                                         walk_fsm, each confirmed before the next. DAMP MAKES THE
                                         ROBOT LIMP: only with it on the gantry or held. Starts after
                                         the current state (from stand up: only walk_fsm; nothing if
                                         already in walk_fsm). Velocity is off while it runs;
                                         on success cmd_vel is followed again, on failure the node
                                         stays stopped (~/resume once the robot is fixed).

Safety:
  * Each command is clamped to max_vx / max_vy / max_wz, scaled as a whole so its direction
    survives, then rate limited to max_linear_accel / max_angular_accel.
  * A cmd_vel older than cmd_timeout counts as zero, so a dead publisher stops the robot.
  * Every request carries duration = command_duration: if this node dies, Unitree's controller
    stops by itself after that long.
  * Once stopped, a zero is sent stop_repeats times and then nothing, so this node doesn't hold
    the walking controller while there's nothing to do.

Replies on /api/sport/response are matched by request id; a refused command is logged with its
code, and missing replies while walking are reported. The FSM id and mode are logged once the
service answers (asked every 2 s until then, and again on ~/resume).
"""

import json
import math
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
from unitree_api.msg import Request, Response

# g1_loco_api.hpp
API_GET_FSM_ID = 7001
API_GET_FSM_MODE = 7002
API_SET_FSM_ID = 7101
API_SET_VELOCITY = 7105

# FSM ids whose controller follows SDK velocity commands
SDK_WALK_FSM = (500, 501)
FSM_DAMP = 1
FSM_STAND_UP = 4

ERRORS = {
    3102: "request sent, no reply",
    3104: "reply timed out",
    7301: "loco state not available (is the robot in FSM 500 or 501?)",
    7302: "invalid FSM id",
}


def limit(v, max_v):
    """Clamp (vx, vy, wz): linear part scaled as a whole, so it keeps its direction."""
    vx, vy, wz = v
    max_vx, max_vy, max_wz = max_v
    scale = max(abs(vx) / max_vx if max_vx > 0 else 0.0, abs(vy) / max_vy if max_vy > 0 else 0.0, 1.0)
    return vx / scale, vy / scale, max(-max_wz, min(max_wz, wz))


def slew(current, target, max_step):
    """Move current towards target by at most max_step (linear part as a vector)."""
    dx, dy = target[0] - current[0], target[1] - current[1]
    d = math.hypot(dx, dy)
    k = 1.0 if d <= max_step[0] else max_step[0] / d
    dw = max(-max_step[1], min(max_step[1], target[2] - current[2]))
    return current[0] + k * dx, current[1] + k * dy, current[2] + dw


class LocoBridge(Node):
    def __init__(self):
        super().__init__("g1_loco_bridge")
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        cmd_vel_topic = p("cmd_vel_topic", "/g1/cmd_vel")
        self.rate = p("rate", 20.0)
        self.cmd_timeout = p("cmd_timeout", 0.3)
        self.command_duration = p("command_duration", 0.5)
        self.max_v = (p("max_vx", 0.3), p("max_vy", 0.2), p("max_wz", 0.6))
        self.max_accel = (p("max_linear_accel", 0.5), p("max_angular_accel", 1.5))
        self.stop_repeats = int(p("stop_repeats", 3))
        self.reply_timeout = p("reply_timeout", 1.0)
        self.walk_fsm = int(p("walk_fsm", 501))
        self.fsm_timeout = p("fsm_timeout", 3.0)  # s for damp and stand up to be confirmed
        self.damp_settle = p("damp_settle", 1.0)  # s in damp before standing up
        self.stand_timeout = p("stand_timeout", 15.0)  # s of retrying walk_fsm while it stands up

        self.cmd = None
        self.cmd_time = None
        self.sent = (0.0, 0.0, 0.0)
        self.zeros_left = 0  # zeros still to send before going quiet
        self.stopped = False
        self.pending = {}  # request id -> (api id, time sent)
        self.waiters = {}  # request id -> [event, (code, data)] for blocking calls
        self.lock = threading.Lock()
        self.transition = False  # ~/enter_walk running: no velocity
        self.velocity_reply = None  # time of the last reply to a velocity request
        self.fsm_known = False
        self.last_id = 0

        self.request_pub = self.create_publisher(Request, "/api/sport/request", 10)
        self.command_pub = self.create_publisher(TwistStamped, "~/command", 10)
        self.create_subscription(Response, "/api/sport/response", self.on_response, 10)
        self.create_subscription(Twist, cmd_vel_topic, self.on_cmd_vel, 10)
        self.create_service(Trigger, "~/stop", self.on_stop)
        self.create_service(Trigger, "~/resume", self.on_resume)
        # Its own group: it waits for replies, which arrive on the default group's subscription
        self.create_service(Trigger, "~/enter_walk", self.on_enter_walk, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0 / self.rate, self.tick)
        self.create_timer(2.0, lambda: None if self.fsm_known else self.query_fsm())
        self.get_logger().info(
            f"{cmd_vel_topic} -> Unitree loco SetVelocity at up to {self.rate:.0f} Hz; limits "
            f"vx {self.max_v[0]:.2f} vy {self.max_v[1]:.2f} m/s, wz {self.max_v[2]:.2f} rad/s"
        )
        self.query_fsm()

    def request(self, api_id, parameter=None, waiter=None):
        msg = Request()
        with self.lock:
            # Unique and increasing, like LocoClient's uptime in ns
            self.last_id = max(self.last_id + 1, time.monotonic_ns())
            rid = self.last_id
            self.pending[rid] = (api_id, time.monotonic())
            if waiter is not None:
                self.waiters[rid] = waiter
        msg.header.identity.id = rid
        msg.header.identity.api_id = api_id
        if parameter is not None:
            msg.parameter = json.dumps(parameter)
        self.request_pub.publish(msg)

    def call(self, api_id, parameter=None, timeout=1.0):
        """Request and wait for the reply: (code, data), or None if none came."""
        waiter = [threading.Event(), None]
        self.request(api_id, parameter, waiter)
        waiter[0].wait(timeout)
        return waiter[1]

    def get_fsm_id(self):
        reply = self.call(API_GET_FSM_ID)
        if reply is None or reply[0] != 0:
            return None
        try:
            return json.loads(reply[1])["data"]
        except (ValueError, KeyError, TypeError):
            return None

    def set_fsm_until(self, fsm_id, timeout, retry_period=1.0):
        """Ask for fsm_id (again every retry_period) until GetFsmId confirms it. The FSM last read."""
        deadline = time.monotonic() + timeout
        fsm = None
        while time.monotonic() < deadline:
            reply = self.call(API_SET_FSM_ID, {"data": fsm_id})
            if reply is not None and reply[0] != 0:
                self.get_logger().warn(f"SetFsmId({fsm_id}): code {reply[0]} {ERRORS.get(reply[0], '')}")
            retry = min(deadline, time.monotonic() + retry_period)
            while time.monotonic() < retry:
                time.sleep(0.2)
                fsm = self.get_fsm_id()
                if fsm == fsm_id:
                    return fsm
        return fsm

    def query_fsm(self):
        self.request(API_GET_FSM_ID)
        self.request(API_GET_FSM_MODE)

    def send_velocity(self, v, duration):
        self.request(API_SET_VELOCITY, {"velocity": [float(x) for x in v], "duration": float(duration)})
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "pelvis"
        cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.angular.z = (float(x) for x in v)
        self.command_pub.publish(cmd)

    def on_response(self, msg):
        with self.lock:
            sent = self.pending.pop(msg.header.identity.id, None)
            waiter = self.waiters.pop(msg.header.identity.id, None)
        if sent is None:
            return  # another client's
        api_id, _ = sent
        if waiter is not None:
            waiter[1] = (msg.header.status.code, msg.data)
            waiter[0].set()
            return
        if api_id == API_SET_VELOCITY:
            self.velocity_reply = time.monotonic()
        code = msg.header.status.code
        if code != 0:
            self.get_logger().warn(
                f"api {api_id} refused, code {code}: {ERRORS.get(code, 'unknown')}", throttle_duration_sec=2.0
            )
        elif api_id in (API_GET_FSM_ID, API_GET_FSM_MODE):
            try:
                value = json.loads(msg.data)["data"]
            except (ValueError, KeyError, TypeError):
                value = msg.data
            name = "FSM id" if api_id == API_GET_FSM_ID else "FSM mode"
            self.fsm_known = True
            if api_id == API_GET_FSM_ID and value not in SDK_WALK_FSM:
                self.get_logger().warn(
                    f"FSM id {value}: velocity commands will likely be ignored. They're followed in "
                    f"{' or '.join(map(str, SDK_WALK_FSM))}; on firmware 1.5.x go damp (1) -> stand up (4) -> 501"
                )
            else:
                self.get_logger().info(f"{name}: {value}")

    def on_cmd_vel(self, msg):
        self.cmd, self.cmd_time = (msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic()

    def on_stop(self, request, response):
        self.stopped = True
        self.sent = (0.0, 0.0, 0.0)
        self.zeros_left = self.stop_repeats
        self.send_velocity(self.sent, self.command_duration)
        response.success = True
        response.message = "stopped; cmd_vel ignored until ~/resume"
        self.get_logger().warn(response.message)
        return response

    def on_enter_walk(self, request, response):
        fsm = self.get_fsm_id()
        if fsm is None:
            response.success = False
            response.message = "no FSM id from the loco service"
            return response
        if fsm == self.walk_fsm:
            response.success = True
            response.message = f"already in FSM {fsm}"
            return response

        steps = [
            (FSM_DAMP, "damp", self.fsm_timeout),
            (FSM_STAND_UP, "stand up", self.fsm_timeout),
            (self.walk_fsm, "walk", self.stand_timeout),
        ]
        ids = [step[0] for step in steps]
        if fsm in ids:  # part way already: don't damp a standing robot again
            steps = steps[ids.index(fsm) + 1 :]
        self.get_logger().warn(
            f"FSM {fsm} -> " + " -> ".join(name for _, name, _ in steps)
            + (": the robot goes limp first" if steps[0][0] == FSM_DAMP else "")
        )
        self.transition = True
        self.stopped = True
        self.sent = (0.0, 0.0, 0.0)
        self.zeros_left = 0
        try:
            for fsm_id, name, timeout in steps:
                fsm = self.set_fsm_until(fsm_id, timeout)
                if fsm != fsm_id:
                    response.success = False
                    response.message = (
                        f"{name} ({fsm_id}) not confirmed after {timeout:.0f} s, FSM is {fsm}; "
                        "velocity stays off until ~/resume"
                    )
                    self.get_logger().error(response.message)
                    return response
                self.get_logger().info(f"FSM {fsm_id} ({name})")
                if fsm_id == FSM_DAMP:
                    time.sleep(self.damp_settle)
        finally:
            self.transition = False
        self.stopped = False
        self.cmd = self.cmd_time = None  # wait for a fresh cmd_vel
        response.success = True
        response.message = f"in FSM {self.walk_fsm}, following cmd_vel"
        self.get_logger().info(response.message)
        return response

    def on_resume(self, request, response):
        self.stopped = False
        self.cmd = self.cmd_time = None  # wait for a fresh cmd_vel
        response.success = True
        response.message = "following cmd_vel"
        self.get_logger().info(response.message)
        self.query_fsm()
        return response

    def tick(self):
        now = time.monotonic()
        # Forget requests that will never be answered
        with self.lock:
            for rid, (_, t) in list(self.pending.items()):
                if now - t > 5.0:
                    del self.pending[rid]
                    self.waiters.pop(rid, None)
        if self.transition:
            return

        cmd, cmd_time = self.cmd, self.cmd_time  # services may clear them meanwhile
        fresh = cmd is not None and cmd_time is not None and now - cmd_time <= self.cmd_timeout
        target = limit(cmd, self.max_v) if fresh and not self.stopped else (0.0, 0.0, 0.0)
        dt = 1.0 / self.rate
        v = slew(self.sent, target, (self.max_accel[0] * dt, self.max_accel[1] * dt))
        if self.stopped:
            v = (0.0, 0.0, 0.0)

        moving = any(abs(x) > 1e-4 for x in v)
        if moving:
            self.zeros_left = self.stop_repeats
        elif self.zeros_left > 0:
            self.zeros_left -= 1
            v = (0.0, 0.0, 0.0)
        else:
            self.sent = (0.0, 0.0, 0.0)
            return  # nothing to do: stay quiet
        self.sent = v
        self.send_velocity(v, self.command_duration)

        with self.lock:
            oldest = min((t for api, t in self.pending.values() if api == API_SET_VELOCITY), default=None)
        if moving and oldest is not None and now - oldest > self.reply_timeout and (
            self.velocity_reply is None or now - self.velocity_reply > self.reply_timeout
        ):
            self.get_logger().warn(
                "no reply from the loco service for over "
                f"{self.reply_timeout:.1f} s: is /api/sport/request reaching the robot?",
                throttle_duration_sec=2.0,
            )


def main():
    # Our own handlers instead of rclpy's: the context stays up long enough to send a zero
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = LocoBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.sent != (0.0, 0.0, 0.0) or node.zeros_left > 0:
            node.send_velocity((0.0, 0.0, 0.0), node.command_duration)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
