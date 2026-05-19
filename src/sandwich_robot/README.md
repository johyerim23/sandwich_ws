# Lecture 7 — MoveIt2

This lecture demonstrates motion planning and collision avoidance with the Franka Panda robot using MoveIt2.

---

## Prerequisites

Install MoveIt2 for ROS 2 Humble:

```bash
sudo apt install ros-humble-moveit
```

Reference: [MoveIt2 Getting Started](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)

---

## MoveIt config files

SRDF, kinematics, joint limits, and planning configs are loaded from the apt-installed package `moveit_resources_panda_moveit_config`:

```bash
cd $(ros2 pkg prefix moveit_resources_panda_moveit_config)/share/moveit_resources_panda_moveit_config
```

---

## Package layout

```
07_moveit2/                     # ROS2 package: moveit2
├── launch/
│   ├── demo.launch.py          # fake hardware + RViz
│   ├── sim.launch.py           # Gazebo + MoveIt2 (collision avoidance)
│   └── pick_and_place.launch.py# Gazebo + MoveIt2 (pick and place)
├── config/
├── urdf/
├── worlds/
│   ├── table.sdf               # table + red obstacle wall
│   └── pick_and_place.sdf      # table + green target box + blue place marker
├── scripts/
│   ├── scene_publisher.py      # publishes table + obstacle to MoveIt planning scene
│   ├── plan_and_execute.py     # cycles between joint-space and Cartesian EE goals
│   ├── pick_and_place.py       # full pick and place sequence with gripper
│   └── print_ee_pose.py        # prints current EE pose (translation + quaternion)
└── README.md
```

---

## Build

```bash
cd ~/ros2_ws
rosdep install -r --from-paths src/07_moveit2 --ignore-src --rosdistro humble -y
colcon build --symlink-install --packages-select moveit2
source install/setup.bash
```

---

## Quickstart demo (fake hardware)

Runs the Panda in RViz with `mock_components/GenericSystem` — motion planning works without physics simulation:

```bash
ros2 launch moveit2 demo.launch.py
```

Reference: [MoveIt2 Quickstart in RViz](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html)

---

## Collision avoidance demo

Launches Gazebo with a table and a red obstacle wall. The planning scene is automatically populated with matching collision objects:

```bash
# Terminal 1
ros2 launch moveit2 sim.launch.py

# Terminal 2 — cycles between a joint-space goal and a Cartesian EE pose goal
ros2 run moveit2 plan_and_execute.py
```

Goals in `plan_and_execute.py`:
- **Joint goal** — arm swept left of the obstacle wall (`joint1 = +1.0 rad`)
- **Pose goal** — EE at `[0.559, -0.059, 0.972]` with fixed orientation, in front of the wall

To read the current EE pose:
```bash
# Formatted translation + quaternion (polls every 0.5 s)
ros2 run moveit2 print_ee_pose.py

# Raw TF2 output
ros2 run tf2_ros tf2_echo world panda_link8
```

---

## Pick and place demo

Launches Gazebo with a table, a green target box (pick), and a blue marker (place):

```bash
# Terminal 1
ros2 launch moveit2 pick_and_place.launch.py

# Terminal 2 — runs the full pick and place sequence
ros2 run moveit2 pick_and_place.py
```

Sequence:
1. Open gripper → pre-grasp (OMPL) → approach (Cartesian) → close gripper → attach object
2. Lift (Cartesian) → transfer to place (OMPL) → place (Cartesian) → open gripper → detach → retreat (Cartesian) → home
3. Return: pre-grasp from place (OMPL) → approach (Cartesian) → close gripper → attach → lift (Cartesian) → transfer back (OMPL) → place at origin (Cartesian) → open gripper → detach → retreat (Cartesian) → home

Planner strategy mirrors MoveIt Task Constructor:
- **Cartesian path** (`/compute_cartesian_path`) for straight-line EE motions (approach, lift, retreat)
- **OMPL** (`/move_action`, IK → joint goal) for free-space transfers
- **JointTrajectoryController** for the gripper (both fingers)

---

## MoveIt Setup Assistant

To create a MoveIt config for a new robot:

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

Reference: [MoveIt Setup Assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)
