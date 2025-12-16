#include <memory>
#include <thread>
#include <chrono> // Für std::chrono::seconds

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Interfaces
#include "my_robot_interfaces/srv/move_to_xyz.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/msg/position_constraint.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

using MoveToXYZ = my_robot_interfaces::srv::MoveToXYZ;
using MoveGroup = moveit_msgs::action::MoveGroup;
using GoalHandleMoveGroup = rclcpp_action::ClientGoalHandle<MoveGroup>;

class MoveXYZServer : public rclcpp::Node
{
public:
  MoveXYZServer() : Node("move_xyz_server_cpp")
  {
    // WICHTIG: Callback Group erstellen (Reentrant)
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Create the service
    // ÄNDERUNG HIER: rmw_qos_profile_services_default -> rclcpp::ServicesQoS()
    service_ = this->create_service<MoveToXYZ>(
      "move_to_xyz",
      std::bind(&MoveXYZServer::handle_service_request, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), 
      callback_group_
    );

    // Create the action client
    action_client_ = rclcpp_action::create_client<MoveGroup>(this, "/move_action", callback_group_);
    RCLCPP_INFO(this->get_logger(), "Move to Position Service ready: /move_to_xyz");
  }

private:
  rclcpp::Service<MoveToXYZ>::SharedPtr service_;
  rclcpp_action::Client<MoveGroup>::SharedPtr action_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_; // Neue Variable

  void handle_service_request(const std::shared_ptr<MoveToXYZ::Request> request, std::shared_ptr<MoveToXYZ::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Anfrage: X=%.2f, Y=%.2f, Z=%.2f", request->x, request->y, request->z);

    // Check if the /move_action client is available
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      response->success = false;
      response->message = "MoveIt Action Server not reachable.";
      RCLCPP_ERROR(this->get_logger(), "Action Server not found.");
      return;
    }

    // Compose the message send to the /move_action client
    // Further information about the message structure can be found here: 
    // https://docs.ros.org/en/noetic/api/moveit_msgs/html/action/MoveGroup.html

    // 1. Define planning parameters and group
    auto goal_msg = MoveGroup::Goal();
    // Groups can be found at: robotic_arm_project/src/robot_moveit/config/robot.srdf.xacro
    goal_msg.request.group_name = "arm"; 

    goal_msg.request.num_planning_attempts = 10;
    goal_msg.request.allowed_planning_time = 5.0;

    // 2. Define the solution contraints (here as a sphere)
    moveit_msgs::msg::PositionConstraint pos_constraint;
    // Define the reference frame and target link
    pos_constraint.header.frame_id = "base_link";
    pos_constraint.link_name = "ee_link";        
    pos_constraint.weight = 1.0;

    // Create the sphere with radius 0.05
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    primitive.dimensions = {0.005}; 

    // Here we define the target pose of the target sphere
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = request->x;
    target_pose.position.y = request->y;
    target_pose.position.z = request->z;
    // Orientation can be left out because its a sphere

    // Put the message together
    pos_constraint.constraint_region.primitives.push_back(primitive);
    pos_constraint.constraint_region.primitive_poses.push_back(target_pose);

    moveit_msgs::msg::Constraints constraints;
    constraints.position_constraints.push_back(pos_constraint);
    goal_msg.request.goal_constraints.push_back(constraints);

    // Send the message to the action client 
    RCLCPP_INFO(this->get_logger(), "Send Goal to MoveIt Client");
    auto send_goal_options = rclcpp_action::Client<MoveGroup>::SendGoalOptions();
    auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);

    // Wait for the result of the action client
    // .get() blockiert hier, aber dank CallbackGroup/MultiThreadedExecutor geht das jetzt!
    auto goal_handle = goal_handle_future.get();

    if (!goal_handle) {
      response->success = false;
      response->message = "Goal was rejected (server not running or goal invalid)";
      RCLCPP_ERROR(this->get_logger(), "Goal invalid.");
      return;
    }

    auto result_future = action_client_->async_get_result(goal_handle);
    
    // Hier ist dein 15-Sekunden-Check
    if (result_future.wait_for(std::chrono::seconds(15)) != std::future_status::ready) {
        response->success = false;
        response->message = "Timeout during the movement.";
        RCLCPP_ERROR(this->get_logger(), "Timeout: Movement took longer than 15s");
        return;
    }

    // Evaluate the result
    auto result = result_future.get();

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->error_code.val == 1) {
        response->success = true;
        response->message = "Success";
        RCLCPP_INFO(this->get_logger(), "GOAL REACHED!");
    } else {
        response->success = false;
        response->message = "ERROR (Code: " + std::to_string(result.result->error_code.val) + ")";
        RCLCPP_ERROR(this->get_logger(), "ERROR DURING MOVEMENT: %d", result.result->error_code.val);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveXYZServer>();

  // Multithread is activated here because both the service and the client need a thread
  // Wir geben 4 Threads an, um sicherzugehen
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
