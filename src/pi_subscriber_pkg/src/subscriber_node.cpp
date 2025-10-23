#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

void topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("pi_subscriber_node"), "I heard: '%s'", msg->data.c_str());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pi_subscriber_node");
  auto subscription = node->create_subscription<std_msgs::msg::String>(
    "chat_topic", 10, topic_callback);
  
  RCLCPP_INFO(node->get_logger(), "Pi Subscriber Node started. Waiting for messages...");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
