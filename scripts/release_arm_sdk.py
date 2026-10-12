#!/usr/bin/env python3
"""Hand the G1's arms and waist back to Unitree's controller.

Holds the currently measured upper-body pose on rt/arm_sdk while ramping the blend weight to 0.
Use it if the controller manager died without releasing the arms (e.g. it was killed with -9).
"""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_hg.msg import LowCmd, LowState

WEIGHT_INDEX = 29
WAIST = range(12, 15)
ARMS = [15, 16, 17, 18, 22, 23, 24, 25]
WRISTS = [19, 20, 21, 26, 27, 28]
# Same gains as urdf/g1.ros2_control.xacro
GAINS = {**{i: (240.0, 6.0) for i in WAIST}, **{i: (300.0, 6.0) for i in ARMS}, **{i: (80.0, 2.0) for i in WRISTS}}


class Release(Node):
    def __init__(self, duration, start_weight):
        super().__init__("release_arm_sdk")
        self.state = None
        self.steps = max(1, int(duration / 0.01))
        self.start_weight = start_weight
        self.step = 0
        self.create_subscription(LowState, "lowstate", self.on_state, qos_profile_sensor_data)
        self.pub = self.create_publisher(LowCmd, "arm_sdk", 10)
        self.timer = self.create_timer(0.01, self.tick)

    def on_state(self, msg):
        self.state = msg

    def tick(self):
        if self.state is None:
            self.get_logger().info("Waiting for lowstate...", throttle_duration_sec=1.0)
            return
        cmd = LowCmd()
        cmd.mode_machine = self.state.mode_machine
        for index, (kp, kd) in GAINS.items():
            motor = cmd.motor_cmd[index]
            motor.mode = 1
            motor.q = self.state.motor_state[index].q
            motor.kp = kp
            motor.kd = kd
        self.step += 1
        weight = self.start_weight * max(0.0, 1.0 - self.step / self.steps)
        cmd.motor_cmd[WEIGHT_INDEX].q = weight
        self.pub.publish(cmd)
        if weight <= 0.0:
            self.get_logger().info("Arm SDK weight is 0; Unitree's controller has the arms.")
            raise SystemExit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=2.0, help="ramp time in seconds")
    parser.add_argument("--start-weight", type=float, default=1.0)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Release(args.duration, args.start_weight)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
