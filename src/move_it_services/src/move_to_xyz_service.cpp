#include <memory>
#include <thread>
#include <chrono> 

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Service and action interfaces used to bridge a simple XYZ request
// to a MoveIt MoveGroup action call.
#include "my_robot_interfaces/srv/move_to_xyz.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/msg/position_constraint.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

using MoveToXYZ = my_robot_interfaces::srv::MoveToXYZ;
using MoveGroup = moveit_msgs::action::MoveGroup;
using GoalHandleMoveGroup = rclcpp_action::ClientGoalHandle<MoveGroup>;

/**
 * Node providing a synchronous XYZ motion service on top of MoveIt.
 *
 * Intent:
 * - Expose a simple service-based interface (`/move_to_xyz`)
 * - Internally translate it into a MoveGroup action request
 *
 * Design assumption:
 * - The robot is already configured in MoveIt
 * - Planning group "arm" with the last link "ee_link" need to be defined in the robot.srdf
 */

class MoveXYZServer : public rclcpp::Node
{
public:
  MoveXYZServer() : Node("move_xyz_server_cpp")
  {
    /**
     * Reentrant group prevents deadlock when synchronously calling 
     * actions from within a service callback.
     */
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    /**
     * Creating the actual service using the custom MoveToXYZ interface 
     * The interface can be found in the my_robot_interfaces package 
     */
    service_ = this->create_service<MoveToXYZ>(
      "move_to_xyz",
      std::bind(&MoveXYZServer::handle_service_request, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), 
      callback_group_
    );

    /**
     * Create the action client connected to the MoveGroup action server 
     * Assumes that MoveGroup action server is available under "/move_action".
     */
    action_client_ = rclcpp_action::create_client<MoveGroup>(this, "/move_action", callback_group_);
    RCLCPP_INFO(this->get_logger(), "Move to Position Service ready: /move_to_xyz");
  }

private:
  // Declare the ROS interfaces used by the node 
  rclcpp::Service<MoveToXYZ>::SharedPtr service_;
  rclcpp_action::Client<MoveGroup>::SharedPtr action_client_;

  // Shared callback group to allow service + action processing at the same time
  rclcpp::CallbackGroup::SharedPtr callback_group_; 

  /**
   * Service callback translating XYZ requests into MoveIt goals.
   * - This function blocks while waiting for the action result
   * - Safe only because of reentrant callback group + multithreaded executor
   */
  void handle_service_request(const std::shared_ptr<MoveToXYZ::Request> request, std::shared_ptr<MoveToXYZ::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Request: X=%.2f, Y=%.2f, Z=%.2f", request->x, request->y, request->z);

    /**
     * Check if the /move_action client is available and fail fast when its not
     * Necessary to not wait indefinetly
     */
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      response->success = false;
      response->message = "MoveIt Action Server not reachable.";
      RCLCPP_ERROR(this->get_logger(), "Action Server not found.");
      return;
    }

    /**
     * Compose MoveGroup goal.
     * Documentation reference is important because the message is large
     * and non-trivial:
     * https://docs.ros.org/en/noetic/api/moveit_msgs/html/action/MoveGroup.html
     */
    auto goal_msg = MoveGroup::Goal();

    /**
     * Planning group name must match the MoveIt SRDF configuration
     * Found here: robotic_arm_project/src/robot_moveit/config/robot.srdf.xacro
     */
    goal_msg.request.group_name = "arm"; 

    // Planning parameters tuned for reliability over speed
    goal_msg.request.num_planning_attempts = 10;
    goal_msg.request.allowed_planning_time = 5.0;

    /**
     * Position constraint:
     * - We use a sphere to allow tolerance instead of a single exact pose
     * - Orientation is intentionally unconstrained because the robot only has 3 DoF
     */
    moveit_msgs::msg::PositionConstraint pos_constraint;

    // Define the reference frame and target link
    pos_constraint.header.frame_id = "base_link";
    pos_constraint.link_name = "ee_link";        
    pos_constraint.weight = 1.0;

    // Spherical constraint region (radius = 5 mm)
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    primitive.dimensions = {0.005}; 

    // Target position comes directly from the service request
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = request->x;
    target_pose.position.y = request->y;
    target_pose.position.z = request->z;

    // Assemble constraint message
    pos_constraint.constraint_region.primitives.push_back(primitive);
    pos_constraint.constraint_region.primitive_poses.push_back(target_pose);

    moveit_msgs::msg::Constraints constraints;
    constraints.position_constraints.push_back(pos_constraint);
    goal_msg.request.goal_constraints.push_back(constraints);

    /**
     * Send goal asynchronously.
     * Blocking happens later when explicitly waiting for the result.
     */
    RCLCPP_INFO(this->get_logger(), "Send Goal to MoveIt Client");
    auto send_goal_options = rclcpp_action::Client<MoveGroup>::SendGoalOptions();
    auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);

    /**
     * Blocking wait for goal acceptance.
     * Safe due to executor configuration.
     */
    auto goal_handle = goal_handle_future.get();

    if (!goal_handle) {
      response->success = false;
      response->message = "Goal was rejected (server not running or goal invalid)";
      RCLCPP_ERROR(this->get_logger(), "Goal invalid.");
      return;
    }

    auto result_future = action_client_->async_get_result(goal_handle);
    
    /**
     * Timeout guard:
     * Prevents the service from hanging indefinitely if planning or execution
     * stalls inside MoveIt.
     */
    if (result_future.wait_for(std::chrono::seconds(15)) != std::future_status::ready) {
        response->success = false;
        response->message = "Timeout during the movement.";
        RCLCPP_ERROR(this->get_logger(), "Timeout: Movement took longer than 15s");
        return;
    }

    // Interpret the MoveIt result
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

  /**
   * WHY multithreaded executor:
   * - Service callback blocks
   * - Action client requires executor threads to process responses
   * Thread count > 1 is mandatory to avoid deadlock.
   */
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
