#!/usr/bin/env python3
"""
Print the current end-effector pose (panda_link8 in world frame).

Usage:
    ros2 run moveit2 print_ee_pose.py
"""
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class PrintEEPose(Node):

    def __init__(self):
        super().__init__('print_ee_pose')
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(0.5, self._lookup)

    def _lookup(self):
        try:
            tf = self._tf_buffer.lookup_transform('world', 'panda_link8', rclpy.time.Time())
        except Exception:
            return

        t = tf.transform.translation
        r = tf.transform.rotation
        self.get_logger().info(
            f'\n'
            f'  Translation : x={t.x:.4f}  y={t.y:.4f}  z={t.z:.4f}\n'
            f'  Rotation    : x={r.x:.4f}  y={r.y:.4f}  z={r.z:.4f}  w={r.w:.4f}'
        )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PrintEEPose())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
