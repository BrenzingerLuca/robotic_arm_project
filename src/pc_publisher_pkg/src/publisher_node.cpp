#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pc_publisher_node");
  auto publisher = node->create_publisher<std_msgs::msg::String>("chat_topic", 10);

  std_msgs::msg::String message;
  size_t count = 0;

  rclcpp::WallRate loop_rate(1.0); // 1 Hz

  while (rclcpp::ok()) {
    message.data = "Hello from PC: " + std::to_string(count++);
    RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
