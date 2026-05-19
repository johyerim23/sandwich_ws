#!/usr/bin/env python3
"""
Plan and execute motions on the Panda arm via the /move_action action server.

Cycles through two goal types to demonstrate collision avoidance:
  1. Joint-space goal  — arm swept to the left of the obstacle wall
  2. Cartesian EE pose — in front of the obstacle wall with fixed orientation

Usage (after sim.launch.py is running):
    ros2 run moveit2 plan_and_execute.py
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

PLANNING_GROUP = 'panda_arm'
EE_LINK = 'panda_link8'
FRAME = 'world'

JOINT_NAMES = [
    'panda_joint1', 'panda_joint2', 'panda_joint3',
    'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7',
]


# ---------------------------------------------------------------------------
# Goal builders
# ---------------------------------------------------------------------------

def joint_goal(values, tol=0.01):
    """Joint-space goal: constrain each joint to a target position."""
    c = Constraints()
    for name, val in zip(JOINT_NAMES, values):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = float(val)
        jc.tolerance_above = tol
        jc.tolerance_below = tol
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    return c


def pose_goal(x, y, z, qx, qy, qz, qw, pos_tol=0.05, ori_tol=0.1):
    """Cartesian EE pose goal with position and orientation constraints."""
    c = Constraints()

    # Position constraint
    pc = PositionConstraint()
    pc.header.frame_id = FRAME
    pc.link_name = EE_LINK
    pc.weight = 1.0
    region = SolidPrimitive()
    region.type = SolidPrimitive.SPHERE
    region.dimensions = [pos_tol]
    target = Pose()
    target.position.x = x
    target.position.y = y
    target.position.z = z
    target.orientation.w = 1.0
    bv = BoundingVolume()
    bv.primitives.append(region)
    bv.primitive_poses.append(target)
    pc.constraint_region = bv
    c.position_constraints.append(pc)

    # Orientation constraint
    oc = OrientationConstraint()
    oc.header.frame_id = FRAME
    oc.link_name = EE_LINK
    oc.orientation.x = qx
    oc.orientation.y = qy
    oc.orientation.z = qz
    oc.orientation.w = qw
    oc.absolute_x_axis_tolerance = ori_tol
    oc.absolute_y_axis_tolerance = ori_tol
    oc.absolute_z_axis_tolerance = ori_tol
    oc.weight = 1.0
    c.orientation_constraints.append(oc)

    return c


# ---------------------------------------------------------------------------
# Goals
# ---------------------------------------------------------------------------

GOAL_JOINT = {
    'label': 'joint goal — left of obstacle (joint1 = +1.0 rad)',
    'constraints': joint_goal([1.0, -0.5, 0.0, -2.0, 0.0, 2.0, 0.785]),
}

GOAL_POSE = {
    'label': 'pose goal — [0.559, -0.059, 0.972] q=[0.924, -0.382, 0.000, -0.000]',
    'constraints': pose_goal(
        x=0.559, y=-0.059, z=0.972,
        qx=0.924, qy=-0.382, qz=0.000, qw=-0.000,
    ),
}

GOALS = [GOAL_JOINT, GOAL_POSE]


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class PlanAndExecute(Node):
    def __init__(self):
        super().__init__('plan_and_execute')
        self._client = ActionClient(self, MoveGroup, '/move_action')
        self._idx = 0
        self._wait_timer = None

        self.get_logger().info('Waiting for /move_action server ...')
        self._client.wait_for_server()
        self.get_logger().info('Connected — starting motion cycle.')
        self._send_goal()

    def _send_goal(self):
        g = GOALS[self._idx % len(GOALS)]
        self.get_logger().info(f'[{self._idx + 1}] {g["label"]}')

        req = MotionPlanRequest()
        req.group_name = PLANNING_GROUP
        # req.planner_id = 'BITstar'  # Default planner is RRTConnect - set in param
        req.goal_constraints.append(g['constraints'])
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options.plan_only = False

        self._client.send_goal_async(goal_msg).add_done_callback(
            self._on_goal_accepted
        )

    def _on_goal_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by move_group')
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        code = future.result().result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('Motion succeeded.')
        else:
            self.get_logger().warn(f'Motion failed — MoveIt error code: {code}')

        self._idx += 1
        self._wait_timer = self.create_timer(2.0, self._on_wait_done)

    def _on_wait_done(self):
        self._wait_timer.cancel()
        self._wait_timer = None
        self._send_goal()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PlanAndExecute())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
