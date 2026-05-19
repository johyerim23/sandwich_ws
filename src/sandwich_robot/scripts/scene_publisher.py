#!/usr/bin/env python3
"""Publishes a MoveIt2 planning scene that mirrors the Gazebo world."""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


def _box(obj_id, frame_id, x, y, z, sx, sy, sz):
    obj = CollisionObject()
    obj.id = obj_id
    obj.header.frame_id = frame_id
    obj.operation = CollisionObject.ADD
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [sx, sy, sz]
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    obj.primitives.append(primitive)
    obj.primitive_poses.append(pose)
    return obj


class ScenePublisher(Node):
    def __init__(self):
        super().__init__('scene_publisher')
        self._client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        # Poll until move_group is ready, then apply once
        self._timer = self.create_timer(1.0, self._try_apply)

    def _try_apply(self):
        if not self._client.service_is_ready():
            return
        self._timer.cancel()

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [
            # Table the robot sits on (world frame: z=0 to 0.4)
            _box('table', 'world', 0.0, 0.0, 0.2, 0.8, 0.8, 0.4),
            # Thin wall obstacle in front of the arm (world frame: z=0.5 to 0.9)
            _box('obstacle', 'world', 0.4, 0.0, 0.7, 0.06, 0.5, 0.4),
        ]

        req = ApplyPlanningScene.Request()
        req.scene = scene
        self._client.call_async(req).add_done_callback(self._done)

    def _done(self, future):
        self.get_logger().info('Planning scene applied (table + obstacle)')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ScenePublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
