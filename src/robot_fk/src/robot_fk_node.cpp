// ROS2 specific imports
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// TF specific imports
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

/**
 * ForwardKinematicsNode
 *
 * Intent:
 * - Read the current end-effector pose from the TF tree
 * - Publish it as a PoseStamped message ont the /fk_info topic
 *
 * Design assumptions:
 * - A valid TF chain exists from "world" to "ee_link"
 * - robot_state_publisher is running
 */
class ForwardKinematicsNode : public rclcpp::Node
{
public:
    ForwardKinematicsNode() : Node("fk_reader"), buffer_(this->get_clock()), listener_(buffer_)
    {
        // A timer based approach is used to decouple fk publishing from joint state changes
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ForwardKinematicsNode::read_fk, this)
        );

        // Create a publisher that publishes the endeffector_pose in the world frame
        fk_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("fk_info", 10);
    }

private:

    /**
     * TF infrastructure:
     * - Buffer stores the TF tree
     * - Listener subscribes to /tf and /tf_static
     */
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;

    // Periodic trigger for FK lookup
    rclcpp::TimerBase::SharedPtr timer_;

    // Publishes the computed forward kinematics pose
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fk_pub_;

    void read_fk()
    {
        try {
            // Lookup the transformation from world to the ee_link using the TF Library
            // TimePointZero: Requests the latest available transform
            geometry_msgs::msg::TransformStamped t =
                buffer_.lookupTransform("world", "ee_link", tf2::TimePointZero);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "world";

        // Translation: directly represents end-effector position
        pose_msg.pose.position.x = t.transform.translation.x;
        pose_msg.pose.position.y = t.transform.translation.y;
        pose_msg.pose.position.z = t.transform.translation.z;

        // Orientation (Quaternion)
        pose_msg.pose.orientation = t.transform.rotation;

        // Publish FK result
        fk_pub_->publish(pose_msg);
        }

        catch (tf2::TransformException &ex){
        }
    }
};

int main(int argc, char **argv)
{
    // Standard ROS 2 node lifecycle.
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForwardKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
