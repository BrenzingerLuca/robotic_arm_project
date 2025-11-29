 #include <rclcpp/rclcpp.hpp>
 #include <tf2_ros/transform_listener.h>
 #include <tf2_ros/buffer.h>
 #include <geometry_msgs/msg/transform_stamped.hpp>
 #include <geometry_msgs/msg/pose_stamped.hpp>

class ForwardKinematicsNode : public rclcpp::Node
{
public:
    ForwardKinematicsNode() : Node("fk_reader"), buffer_(this->get_clock()), listener_(buffer_)
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ForwardKinematicsNode::read_fk, this)
        );

        fk_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("fk_info", 10);
    }

private:

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fk_pub_;

    void read_fk()
    {
        try {
            geometry_msgs::msg::TransformStamped t =
                buffer_.lookupTransform("world", "first_arm", tf2::TimePointZero);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "world";

        // Translation
        pose_msg.pose.position.x = t.transform.translation.x;
        pose_msg.pose.position.y = t.transform.translation.y;
        pose_msg.pose.position.z = t.transform.translation.z;

        // Rotation (Quaternion)
        pose_msg.pose.orientation = t.transform.rotation;

        // Publisher
        fk_pub_->publish(pose_msg);

        }

        catch (tf2::TransformException &ex){
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForwardKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
