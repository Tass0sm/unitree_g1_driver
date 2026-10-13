#!/usr/bin/env python3
"""Inspire hand fingers on /joint_states, for robot_state_publisher (the URDF's hand joints).

Subscribes to each hand's inspire_hand.py ~/state (position = angle / 1000, 1 = open, in the order
little, ring, middle, index, thumb_bend, thumb_rotation) and publishes the URDF's driven finger
joints at `rate`; robot_state_publisher fills in the mimic joints. A hand with no state (driver not
running, or no left hand) is shown in open_pose. Angles map linearly onto the URDF joint limits
(open = the end where the finger is extended): approximate, for display only.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# (URDF joint without the L_/R_ prefix, its limits (open, closed) in rad), in ~/state order
JOINTS = [
    ("pinky_proximal_joint", (0.0, 1.7)),
    ("ring_proximal_joint", (0.0, 1.7)),
    ("middle_proximal_joint", (0.0, 1.7)),
    ("index_proximal_joint", (0.0, 1.7)),
    ("thumb_proximal_pitch_joint", (-0.1, 0.6)),
    ("thumb_proximal_yaw_joint", (-0.1, 1.3)),
]


class InspireHandJointStates(Node):
    def __init__(self):
        super().__init__("inspire_hand_joint_states")
        topics = {
            "R": self.declare_parameter("right_state_topic", "/inspire_hand/state").value,
            "L": self.declare_parameter("left_state_topic", "/inspire_hand_left/state").value,
        }
        self.open_pose = list(self.declare_parameter("open_pose", [1.0, 1.0, 1.0, 1.0, 1.0, 0.0]).value)
        self.positions = {side: list(self.open_pose) for side in topics}
        for side, topic in topics.items():
            self.create_subscription(JointState, topic, lambda msg, side=side: self.on_state(side, msg), 10)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / self.declare_parameter("rate", 20.0).value, self.publish)

    def on_state(self, side, msg):
        if len(msg.position) >= len(JOINTS):
            self.positions[side] = list(msg.position[: len(JOINTS)])

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for side, positions in self.positions.items():
            for (joint, (opened, closed)), p in zip(JOINTS, positions):
                p = min(1.0, max(0.0, p))
                msg.name.append(f"{side}_{joint}")
                msg.position.append(closed + p * (opened - closed))
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = InspireHandJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
