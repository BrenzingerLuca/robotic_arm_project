cmake_minimum_required(VERSION 3.8)
project(simple_gpio_servo_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(simple_gpio_servo_node src/simple_gpio_servo_node.cpp)
ament_target_dependencies(simple_gpio_servo_node rclcpp std_msgs)

install(TARGETS
  simple_gpio_servo_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()