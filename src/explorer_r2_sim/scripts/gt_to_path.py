#!/usr/bin/env python3
# Republish ground-truth pose from /ground_truth/pose (TFMessage) as
# /ground_truth/path (nav_msgs/Path) so evo can ingest it the same way it
# reads /ov_msckf/pathimu and /path (FAST_LIO).
#
# Usage:
#   ros2 run explorer_r2_sim gt_to_path.py
#   ros2 run explorer_r2_sim gt_to_path.py --ros-args -p target_frame:=explorer_r2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class GroundTruthToPath(Node):
    def __init__(self):
        super().__init__("gt_to_path")
        self.declare_parameter("target_frame", "explorer_r2")
        self.declare_parameter("fixed_frame",  "world")
        self.target_frame = self.get_parameter("target_frame").value
        self.fixed_frame = self.get_parameter("fixed_frame").value

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.path = Path()
        self.path.header.frame_id = self.fixed_frame

        self.create_subscription(
            TFMessage, "/ground_truth/pose", self._on_tf, qos)
        self.pub = self.create_publisher(
            Path, "/ground_truth/path", qos)

        self.get_logger().info(
            f"Tracking {self.target_frame} in {self.fixed_frame}; "
            "publishing /ground_truth/path")

    def _on_tf(self, msg: TFMessage) -> None:
        for tf in msg.transforms:
            if tf.child_frame_id != self.target_frame:
                continue
            ps = PoseStamped()
            ps.header = tf.header
            ps.pose.position.x = tf.transform.translation.x
            ps.pose.position.y = tf.transform.translation.y
            ps.pose.position.z = tf.transform.translation.z
            ps.pose.orientation = tf.transform.rotation
            self.path.header.stamp = tf.header.stamp
            self.path.poses.append(ps)
            # Keep memory bounded for long runs.
            if len(self.path.poses) > 20000:
                self.path.poses = self.path.poses[-20000:]
            self.pub.publish(self.path)


def main():
    rclpy.init()
    try:
        rclpy.spin(GroundTruthToPath())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
