"""
Gazebo Fortress + MoveIt2 launch for the Franka Panda robot.

Starts Gazebo with a table world, spawns the Panda, activates controllers,
then brings up move_group and RViz for motion planning.
"""

import os
from os import environ
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python import get_search_paths
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    demo_pkg = get_package_share_directory("moveit2")

    # Build resource paths so Gazebo can find package:// → model:// meshes
    gz_resource_paths = os.pathsep.join(
        [os.path.join(p, "share") for p in get_search_paths()]
    )

    # MoveIt config: SRDF/kinematics from panda_moveit_config, URDF from our Gazebo xacro
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path=os.path.join(demo_pkg, "urdf", "panda_gazebo.urdf.xacro")
        )
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # --- Gazebo Fortress ---
    # Launch directly (bypasses gz_sim.launch.py on_exit_shutdown bug)
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", os.path.join(demo_pkg, "worlds", "table.sdf")],
        output="screen",
        additional_env={
            "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": os.pathsep.join([
                environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", ""),
                environ.get("LD_LIBRARY_PATH", ""),
            ]),
            "GZ_SIM_SYSTEM_PLUGIN_PATH": os.pathsep.join([
                environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
                environ.get("LD_LIBRARY_PATH", ""),
            ]),
            "IGN_GAZEBO_RESOURCE_PATH": os.pathsep.join([
                environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
                gz_resource_paths,
            ]),
        },
    )

    # Bridge Gazebo clock → ROS /clock
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Publish robot_description so Gazebo can spawn from it
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
        ],
    )

    # Spawn the Panda into Gazebo from /robot_description
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "panda"],
        output="screen",
    )

    # Controllers — spawned after controller_manager is ready (~8 s)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
        output="screen",
    )
    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller", "-c", "/controller_manager"],
        output="screen",
    )
    controllers = TimerAction(
        period=8.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            hand_controller_spawner,
        ],
    )

    # move_group — after Gazebo + controllers are up (~12 s)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"publish_monitored_planning_scene": True},
        ],
    )

    # RViz — after move_group finishes loading (~16 s)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            os.path.join(demo_pkg, "config", "panda_moveit_config_demo.rviz"),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    scene_publisher = Node(
        package="moveit2",
        executable="scene_publisher.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return [
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        controllers,
        TimerAction(period=12.0, actions=[move_group_node]),
        TimerAction(period=16.0, actions=[rviz_node, scene_publisher]),
    ]
