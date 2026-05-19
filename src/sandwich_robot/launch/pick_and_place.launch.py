"""
Gazebo Fortress + MoveIt2 launch for the Panda pick-and-place demo.

Uses pick_and_place.sdf (table + green target box, no obstacle wall).
The pick_and_place.py script sets up the planning scene and executes the motions.
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

    gz_resource_paths = os.pathsep.join(
        [os.path.join(p, "share") for p in get_search_paths()]
    )

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

    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", os.path.join(demo_pkg, "worlds", "pick_and_place.sdf")],
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

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "panda"],
        output="screen",
    )

    controllers = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["panda_arm_controller", "-c", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["panda_hand_controller", "-c", "/controller_manager"],
                output="screen",
            ),
        ],
    )

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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(demo_pkg, "config", "panda_moveit_config_demo.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return [
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        controllers,
        TimerAction(period=12.0, actions=[move_group_node]),
        TimerAction(period=16.0, actions=[rviz_node]),
    ]
