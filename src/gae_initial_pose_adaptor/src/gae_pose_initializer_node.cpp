#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <component_interface_specs/localization.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

class GaePoseInitializerNode : public rclcpp::Node
{
public:
  GaePoseInitializerNode()
      : Node("gae_pose_initializer_node"),
        tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)), // Updated initialization
        tf_listener_(tf_buffer_)
  {
    this->declare_parameter("map_z", 81.02);
    publisher_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10, std::bind(&GaePoseInitializerNode::initialPoseCallback, this, std::placeholders::_1));

    client_ = this->create_client<localization_interface::Initialize::Service>("/localization/initialize");
  }

private:
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    auto request = std::make_shared<localization_interface::Initialize::Service::Request>();

    geometry_msgs::msg::PoseStamped pose_in_source_frame;
    pose_in_source_frame.header = msg->header;
    pose_in_source_frame.pose = msg->pose.pose;

    geometry_msgs::msg::PoseStamped pose_in_map_frame;

    try
    {
      auto transform_stamped = this->tf_buffer_.lookupTransform("map", pose_in_source_frame.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(pose_in_source_frame, pose_in_map_frame, transform_stamped);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not transform initial pose: %s", ex.what());
      return;
    }

    pose_in_map_frame.pose.position.z = this->get_parameter("map_z").as_double();

    msg->pose.pose = pose_in_map_frame.pose;
    request->pose_with_covariance.push_back(*msg);
    request->method = 1;

    client_->async_send_request(request);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Client<localization_interface::Initialize::Service>::SharedPtr client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GaePoseInitializerNode>());
  rclcpp::shutdown();
  return 0;
}
