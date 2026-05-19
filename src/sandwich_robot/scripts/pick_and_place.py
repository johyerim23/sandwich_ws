#!/usr/bin/env python3
"""
Pick and place demo for the Panda arm.

Mirrors the MoveIt Task Constructor planner strategy:
  - CartesianPath  (/compute_cartesian_path) for straight-line EE motions
                   (approach down, lift up, place down, retreat up)
  - PipelinePlanner (OMPL via /move_action, IK→joint goal) for free-space
                   transfers (pre-grasp, move to place)
  - JointInterpolation (GripperCommand action) for the hand

Sequence:
  ── Pick ──────────────────────────────────────────
  1. Set up planning scene
  2. Open gripper
  3. Pre-grasp  — OMPL: move to above the box
  4. Approach   — Cartesian: straight down to grasp height
  5. Close gripper  →  attach object to EE in planning scene
  6. Lift       — Cartesian: straight up
  7. Transfer   — OMPL: move to above place position
  8. Place      — Cartesian: straight down to place height
  9. Open gripper  →  detach object from EE
 10. Retreat    — Cartesian: straight up
 11. Home       — joint-space: return to ready pose
  ── Return box ────────────────────────────────────
 12. Pre-grasp from place — OMPL: move above place position
 13. Approach from place  — Cartesian: straight down to grasp height
 14. Close gripper  →  attach
 15. Lift              — Cartesian
 16. Transfer back    — OMPL
 17. Approach to origin — Cartesian: straight down to grasp height
 18. Open gripper  →  detach
 19. Retreat           — Cartesian
 20. Home              — joint-space

Usage (after pick_and_place.launch.py is running):
    ros2 run moveit2 pick_and_place.py
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    MotionPlanRequest,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from tf2_ros import Buffer, TransformListener

# ── Constants ──────────────────────────────────────────────────────────────
PLANNING_GROUP = 'panda_arm'
EE_LINK        = 'panda_link8'
HAND_LINK      = 'panda_hand'
FRAME          = 'world'
TOUCH_LINKS    = ['panda_hand', 'panda_leftfinger', 'panda_rightfinger', 'panda_link8']

JOINT_NAMES = [
    'panda_joint1', 'panda_joint2', 'panda_joint3',
    'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7',
]

GRIPPER_OPEN   = 0.07   # m per finger (fully open)
GRIPPER_CLOSE  = 0.025  # m per finger (grasping ~70 mm box — fingers press sides)

# Panda "ready" configuration from SRDF
READY_JOINTS = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]

# Object and place positions (world frame; robot base at z=0.4)
BOX_X, BOX_Y     = 0.55,  0.0
PLACE_X, PLACE_Y = 0.55,  0.35
GRASP_Z          = 0.57   # EE z at grasp   (box top ~0.55, pedestal top ~0.50)
APPROACH_Z       = 0.72   # EE z for pre-grasp / pre-place
LIFT_Z           = 0.82   # EE z while carrying


# ── Goal builders ──────────────────────────────────────────────────────────

def _joint_request(joint_positions, tol=0.01):
    """Joint-space motion plan request (used after IK for OMPL free-space moves)."""
    c = Constraints()
    for name, pos in zip(JOINT_NAMES, joint_positions):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = float(pos)
        jc.tolerance_above = tol
        jc.tolerance_below = tol
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    req = MotionPlanRequest()
    req.group_name = PLANNING_GROUP
    req.goal_constraints.append(c)
    req.num_planning_attempts = 10
    req.allowed_planning_time = 10.0
    req.max_velocity_scaling_factor = 0.3
    req.max_acceleration_scaling_factor = 0.3
    return req


def _gripper_goal(position):
    point = JointTrajectoryPoint()
    point.positions = [position, position]
    point.time_from_start = Duration(sec=2)
    traj = JointTrajectory()
    traj.joint_names = ['panda_finger_joint1', 'panda_finger_joint2']
    traj.points = [point]
    g = FollowJointTrajectory.Goal()
    g.trajectory = traj
    return g


# ── Node ───────────────────────────────────────────────────────────────────

class PickAndPlace(Node):

    STEPS = [
        # ── Pick ──────────────────────────────────────────────────────────
        ('scene',   None,                               'Set up planning scene'),
        ('gripper', GRIPPER_OPEN,                       'Open gripper'),
        ('arm',     (BOX_X,   BOX_Y,   APPROACH_Z),    'Pre-grasp (OMPL)'),
        ('cart',    (BOX_X,   BOX_Y,   GRASP_Z),       'Grasp approach (Cartesian)'),
        ('gripper', GRIPPER_CLOSE,                      'Close gripper'),
        ('attach',  None,                               'Attach object to EE'),
        ('cart',    (BOX_X,   BOX_Y,   LIFT_Z),        'Lift (Cartesian)'),
        ('arm',     (PLACE_X, PLACE_Y, LIFT_Z),        'Transfer to place (OMPL)'),
        ('cart',    (PLACE_X, PLACE_Y, GRASP_Z),       'Place approach (Cartesian)'),
        ('gripper', GRIPPER_OPEN,                       'Open gripper (release)'),
        ('detach',  None,                               'Detach object from EE'),
        ('cart',    (PLACE_X, PLACE_Y, APPROACH_Z),    'Retreat (Cartesian)'),
        ('home',    None,                               'Return to home pose'),
        # ── Return box ────────────────────────────────────────────────────
        ('gripper', GRIPPER_OPEN,                       'Open gripper'),
        ('arm',     (PLACE_X, PLACE_Y, APPROACH_Z),    'Pre-grasp from place (OMPL)'),
        ('cart',    (PLACE_X, PLACE_Y, GRASP_Z),       'Approach from place (Cartesian)'),
        ('gripper', GRIPPER_CLOSE,                      'Close gripper'),
        ('attach',  None,                               'Attach object to EE'),
        ('cart',    (PLACE_X, PLACE_Y, LIFT_Z),        'Lift from place (Cartesian)'),
        ('arm',     (BOX_X,   BOX_Y,   LIFT_Z),         'Transfer back (OMPL)'),
        ('cart',    (BOX_X,   BOX_Y,   GRASP_Z),       'Approach to origin (Cartesian)'),
        ('gripper', GRIPPER_OPEN,                       'Open gripper (release)'),
        ('detach',  None,                               'Detach object from EE'),
        ('cart',    (BOX_X,   BOX_Y,   APPROACH_Z),    'Retreat (Cartesian)'),
        ('home',    None,                               'Return to home pose'),
    ]

    def __init__(self):
        super().__init__('pick_and_place')
        self._arm     = ActionClient(self, MoveGroup,          '/move_action')
        self._exec    = ActionClient(self, ExecuteTrajectory,  '/execute_trajectory')
        self._gripper = ActionClient(self, FollowJointTrajectory, '/panda_hand_controller/follow_joint_trajectory')
        self._scene   = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self._ik      = self.create_client(GetPositionIK,      '/compute_ik')
        self._cart    = self.create_client(GetCartesianPath,   '/compute_cartesian_path')
        self._step    = 0
        self._ori     = None  # (qx, qy, qz, qw) captured from TF2 at startup

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.get_logger().info('Waiting for servers ...')
        self._arm.wait_for_server()
        self._exec.wait_for_server()
        self._gripper.wait_for_server()
        self._scene.wait_for_service()
        self._ik.wait_for_service()
        self._cart.wait_for_service()

        self._tf_timer = self.create_timer(0.5, self._lookup_init_ori)

    def _lookup_init_ori(self):
        try:
            tf = self._tf_buffer.lookup_transform(FRAME, EE_LINK, rclpy.time.Time())
        except Exception:
            return
        self._tf_timer.cancel()
        q = tf.transform.rotation
        self._ori = (q.x, q.y, q.z, q.w)
        self.get_logger().info(
            f'Init EE orientation — x={q.x:.3f} y={q.y:.3f} z={q.z:.3f} w={q.w:.3f}'
        )
        self.get_logger().info('Starting pick and place.')
        self._next()

    # ── Step dispatcher ────────────────────────────────────────────────────

    def _next(self):
        if self._step >= len(self.STEPS):
            self.get_logger().info('Pick and place complete.')
            return

        kind, data, label = self.STEPS[self._step]
        self.get_logger().info(f'[{self._step + 1}/{len(self.STEPS)}] {label}')

        if kind == 'scene':
            self._publish_scene()
        elif kind == 'arm':
            self._send_arm(*data)
        elif kind == 'cart':
            self._send_cart(*data)
        elif kind == 'home':
            self._send_home()
        elif kind == 'gripper':
            self._send_gripper(data)
        elif kind == 'attach':
            self._attach_object()
        elif kind == 'detach':
            self._detach_object()

    def _done(self):
        self._step += 1
        self._next()

    # ── Planning scene ─────────────────────────────────────────────────────

    def _publish_scene(self):
        def _box(obj_id, x, y, z, sx, sy, sz):
            obj = CollisionObject()
            obj.id = obj_id
            obj.header.frame_id = FRAME
            obj.operation = CollisionObject.ADD
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [sx, sy, sz]
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0
            obj.primitives.append(prim)
            obj.primitive_poses.append(pose)
            return obj

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [
            _box('table',          0.0,     0.0,     0.2,   1.4,  1.4,  0.4),
            _box('pick_pedestal',  BOX_X,   BOX_Y,   0.45,  0.12, 0.12, 0.10),
            _box('place_pedestal', PLACE_X, PLACE_Y, 0.45,  0.12, 0.12, 0.10),
            _box('target_box',     BOX_X,   BOX_Y,   0.530, 0.06, 0.06, 0.06),
        ]
        req = ApplyPlanningScene.Request()
        req.scene = scene
        self._scene.call_async(req).add_done_callback(lambda _: self._done())

    # ── OMPL arm motion (IK → joint-space goal) ────────────────────────────

    def _send_arm(self, x, y, z):
        qx, qy, qz, qw = self._ori
        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = PLANNING_GROUP
        ik_req.ik_request.ik_link_name = EE_LINK
        ik_req.ik_request.pose_stamped = ps
        ik_req.ik_request.timeout.sec = 5

        self._ik.call_async(ik_req).add_done_callback(self._ik_done)

    def _ik_done(self, future):
        resp = future.result()
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'IK failed (code {resp.error_code.val}), skipping step')
            self._done()
            return

        js = resp.solution.joint_state
        joint_map = dict(zip(js.name, js.position))
        values = [joint_map[n] for n in JOINT_NAMES]

        goal = MoveGroup.Goal()
        goal.request = _joint_request(values)
        goal.planning_options.plan_only = False
        self._arm.send_goal_async(goal).add_done_callback(self._arm_accepted)

    def _arm_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Arm goal rejected')
            return
        handle.get_result_async().add_done_callback(self._arm_result)

    def _arm_result(self, future):
        code = future.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(f'Arm motion failed (code {code})')
        self._done()

    def _send_home(self):
        goal = MoveGroup.Goal()
        goal.request = _joint_request(READY_JOINTS)
        goal.planning_options.plan_only = False
        self._arm.send_goal_async(goal).add_done_callback(self._arm_accepted)

    # ── Cartesian straight-line EE motion ──────────────────────────────────

    def _send_cart(self, x, y, z):
        qx, qy, qz, qw = self._ori
        target = Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z
        target.orientation.x = qx
        target.orientation.y = qy
        target.orientation.z = qz
        target.orientation.w = qw

        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.group_name = PLANNING_GROUP
        req.link_name = EE_LINK
        req.waypoints = [target]
        req.max_step = 0.01        # 1 cm resolution
        req.jump_threshold = 0.0   # disabled
        req.avoid_collisions = True

        self._cart.call_async(req).add_done_callback(self._cart_done)

    def _cart_done(self, future):
        resp = future.result()
        if resp.fraction < 0.9:
            self.get_logger().warn(f'Cartesian path only {resp.fraction * 100:.0f}% complete')

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = resp.solution
        self._exec.send_goal_async(goal).add_done_callback(self._exec_accepted)

    def _exec_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('ExecuteTrajectory rejected')
            return
        handle.get_result_async().add_done_callback(self._exec_result)

    def _exec_result(self, future):
        code = future.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(f'Cartesian execution failed (code {code})')
        self._done()

    # ── Gripper ────────────────────────────────────────────────────────────

    def _send_gripper(self, position):
        self._gripper.send_goal_async(_gripper_goal(position)).add_done_callback(
            self._gripper_accepted
        )

    def _gripper_accepted(self, future):
        future.result().get_result_async().add_done_callback(
            lambda _: self._done()
        )

    # ── Attach / detach ────────────────────────────────────────────────────

    def _attach_object(self):
        aco = AttachedCollisionObject()
        aco.link_name = HAND_LINK
        aco.object.id = 'target_box'
        aco.object.header.frame_id = FRAME
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = TOUCH_LINKS
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        self._scene.call_async(req).add_done_callback(lambda _: self._done())

    def _detach_object(self):
        aco = AttachedCollisionObject()
        aco.link_name = HAND_LINK
        aco.object.id = 'target_box'
        aco.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        self._scene.call_async(req).add_done_callback(lambda _: self._done())


# ── Entry point ────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PickAndPlace())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
