cmake_minimum_required(VERSION 3.8)
project(sandwich_robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/scene_publisher.py
  scripts/plan_and_execute.py
  scripts/pick_and_place.py
  scripts/print_ee_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
