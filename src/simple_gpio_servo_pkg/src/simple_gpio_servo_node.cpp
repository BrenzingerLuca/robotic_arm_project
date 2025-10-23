#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <string>
#include <thread>
#include <chrono>

#define SERVO_GPIO 18       // GPIO Pin für den Servo
#define SERVO_MIN 5         // Min-Position in ms
#define SERVO_MAX 25        // Max-Position in ms
#define SERVO_FREQ 50       // PWM-Frequenz in Hz

class SimpleGpioServoNode : public rclcpp::Node
{
public:
  SimpleGpioServoNode() : Node("simple_gpio_servo_node")
  {
    // GPIO exportieren
    export_gpio(SERVO_GPIO);
    set_gpio_direction(SERVO_GPIO, "out");

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "servo_command", 10,
        std::bind(&SimpleGpioServoNode::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Simple GPIO Servo Node started on GPIO %d.", SERVO_GPIO);
  }

  ~SimpleGpioServoNode()
  {
    unexport_gpio(SERVO_GPIO);
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg)
  {
    int pulse_ms = SERVO_MIN;

    if (msg->data == "max") pulse_ms = SERVO_MAX;
    else if (msg->data == "mid") pulse_ms = (SERVO_MIN + SERVO_MAX) / 2;
    else if (msg->data == "min") pulse_ms = SERVO_MIN;

    RCLCPP_INFO(this->get_logger(), "Command: '%s' → Pulse %d ms", msg->data.c_str(), pulse_ms);

    // Einfacher PWM-Loop (Blockierend, kurze Dauer)
    int period_us = 1000000 / SERVO_FREQ;
    int high_time_us = pulse_ms * 1000;

    // einmaliges Puls-Setzen
    write_gpio(SERVO_GPIO, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(high_time_us));
    write_gpio(SERVO_GPIO, 0);
    std::this_thread::sleep_for(std::chrono::microseconds(period_us - high_time_us));
  }

  void export_gpio(int gpio)
  {
    std::ofstream("/sys/class/gpio/export") << gpio;
  }

  void unexport_gpio(int gpio)
  {
    std::ofstream("/sys/class/gpio/unexport") << gpio;
  }

  void set_gpio_direction(int gpio, const std::string &dir)
  {
    std::ofstream("/sys/class/gpio/gpio" + std::to_string(gpio) + "/direction") << dir;
  }

  void write_gpio(int gpio, int value)
  {
    std::ofstream("/sys/class/gpio/gpio" + std::to_string(gpio) + "/value") << value;
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleGpioServoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

