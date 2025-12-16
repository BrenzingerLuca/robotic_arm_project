cmake_minimum_required(VERSION 3.8)
project(move_it_services)

# Compiler-Optionen für sauberen Code
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Abhängigkeiten finden
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED) # Dein Interface Paket

# 2. Executable erstellen
# Name des Nodes: move_to_xyz_service
# Quelldatei: src/move_to_xyz_service.cpp
add_executable(move_to_xyz_service src/move_to_xyz_service.cpp)

# 3. Libraries linken
ament_target_dependencies(move_to_xyz_service
  rclcpp
  rclcpp_action
  moveit_msgs
  geometry_msgs
  shape_msgs
  my_robot_interfaces
)

# 4. Installieren (damit ros2 run ihn findet)
install(TARGETS
  move_to_xyz_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()