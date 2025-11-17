#include "gnss_poser/gnss_poser_core.hpp"

#include <geography_utils/height.hpp>
#include <geography_utils/projection.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

// #include <tf2_geometry_msgs/tf2_geometry_msgs.h> // for ros2 foxy compatibility

#include <algorithm>
#include <memory>
#include <string>
#include <cmath>
#include <vector>

namespace gnss_poser
{
  GNSSPoser::GNSSPoser(const rclcpp::NodeOptions &node_options)
      : rclcpp::Node("gnss_poser", node_options),
        tf2_listener_(tf2_buffer_),
        tf2_broadcaster_(*this),
        base_frame_(declare_parameter<std::string>("base_frame")),
        gnss_base_frame_(declare_parameter<std::string>("gnss_base_frame")),
        map_frame_(declare_parameter<std::string>("map_frame")),
        first_navsat_msg(declare_parameter<std::string>("first_navsat_msg", "nav_sat_fix")),
        second_navsat_msg(declare_parameter<std::string>("second_navsat_msg", "nav_sat_fix2")),
        publish_topic_(declare_parameter<std::string>("publish_topic", "gnss_pose_cov")),
        publish_topic_pose_stamped_(declare_parameter<std::string>("publish_topic_pose_stamped", "gnss_pose_stamped"))
  {
    const auto adaptor = component_interface_utils::NodeAdaptor(this);
    adaptor.init_sub(
        sub_map_projector_info_,
        [this](const MapProjectorInfo::Message::ConstSharedPtr msg)
        { callbackMapProjectorInfo(msg); });

    rclcpp::QoS qos(10);

    sub_navsat1_.subscribe(this, first_navsat_msg, qos.get_rmw_qos_profile());
    sub_navsat2_.subscribe(this, second_navsat_msg, qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), sub_navsat1_, sub_navsat2_);

    sync_->setMaxIntervalDuration(rclcpp::Duration(std::chrono::milliseconds(100)));
    sync_->registerCallback(&GNSSPoser::syncCallback, this);

    pose_cov_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(publish_topic_, rclcpp::QoS{1});
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(publish_topic_pose_stamped_, rclcpp::QoS{1});

    RCLCPP_INFO(
        this->get_logger(), "GNSSPoser initialized with base_frame: %s, gnss_base_frame: %s, map_frame: %s, publish_topic: %s, publish_topic_pose_stamped: %s",
        base_frame_.c_str(), gnss_base_frame_.c_str(), map_frame_.c_str(),
        publish_topic_.c_str(), publish_topic_pose_stamped_.c_str());
  }

  void GNSSPoser::syncCallback(
      const sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_fix_msg_ptr_1,
      const sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_fix_msg_ptr_2)
  {
    if (!received_map_projector_info_)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
          "map_projector_info has not been received yet.");
      return;
    }

    if (!isFixed(nav_sat_fix_msg_ptr_1->status) || !isFixed(nav_sat_fix_msg_ptr_2->status))
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
          "One or both GNSS signals are not fixed.");
      return;
    }

    geographic_msgs::msg::GeoPoint gps_point_1, gps_point_2;
    gps_point_1.latitude = nav_sat_fix_msg_ptr_1->latitude;
    gps_point_1.longitude = nav_sat_fix_msg_ptr_1->longitude;
    gps_point_1.altitude = nav_sat_fix_msg_ptr_1->altitude;

    gps_point_2.latitude = nav_sat_fix_msg_ptr_2->latitude;
    gps_point_2.longitude = nav_sat_fix_msg_ptr_2->longitude;
    gps_point_2.altitude = nav_sat_fix_msg_ptr_2->altitude;

    geometry_msgs::msg::Point pos1 = geography_utils::project_forward(gps_point_1, projector_info_);
    geometry_msgs::msg::Point pos2 = geography_utils::project_forward(gps_point_2, projector_info_);

    pos1.z = geography_utils::convert_height(
        pos1.z, gps_point_1.latitude, gps_point_1.longitude, MapProjectorInfo::Message::WGS84,
        projector_info_.vertical_datum);
    if (std::isnan(pos1.z))
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
          "Height conversion failed for first GNSS point.");
      pos1.z = 0.0; // Default to zero if conversion fails
    }

    pos2.z = geography_utils::convert_height(
        pos2.z, gps_point_2.latitude, gps_point_2.longitude, MapProjectorInfo::Message::WGS84,
        projector_info_.vertical_datum);

    if (std::isnan(pos2.z))
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
          "Height conversion failed for second GNSS point.");
      pos2.z = 0.0; // Default to zero if conversion fails
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, std::atan2(pos1.y - pos2.y, pos1.x - pos2.x));
    q.normalize();
    geometry_msgs::msg::Quaternion orientation = tf2::toMsg(q);

    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
    pose_cov_msg.header.stamp = nav_sat_fix_msg_ptr_1->header.stamp;
    pose_cov_msg.header.frame_id = map_frame_;

    pose_cov_msg.pose.pose.position = pos2;
    pose_cov_msg.pose.pose.orientation = orientation;
    double var_x1 = nav_sat_fix_msg_ptr_1->position_covariance[0]; // x variance
    double var_y1 = nav_sat_fix_msg_ptr_1->position_covariance[4]; // y variance
    double var_z1 = nav_sat_fix_msg_ptr_1->position_covariance[8]; // z variance
    double var_x2 = nav_sat_fix_msg_ptr_2->position_covariance[0]; // x variance
    double var_y2 = nav_sat_fix_msg_ptr_2->position_covariance[4]; // y variance
    double var_z2 = nav_sat_fix_msg_ptr_2->position_covariance[8]; // z variance

    double dx = pos2.x - pos1.x;
    double dy = pos2.y - pos1.y;
    double magnitude = dx * dx + dy * dy;

    if (magnitude < 1e-2)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
          "Positions are too close, covariance may not be reliable.");
      magnitude = 1.0; // Avoid division by zero
    }
    // Error propagation for yaw covariance for: std::atan2(dy, dx)
    double yaw_cov = (dy * dy * (var_x1 + var_x2) + dx * dx * (var_y1 + var_y2)) / (magnitude * magnitude);
    // Average x ,y, z variance: A+B -> varA + varB
    double var_x = (var_x1 + var_x2) / 2.0;
    double var_y = (var_y1 + var_y2) / 2.0;
    double var_z = (var_z1 + var_z2) / 2.0;

    // minimum yaw variance threshold
    if (yaw_cov < 1e-6)
    {
      yaw_cov = 1e-6;
    }

    // Fill full 6x6 covariance matrix
    pose_cov_msg.pose.covariance = {
        var_x, 0.0, 0.0, 0.0, 0.0, 0.0,  // position x
        0.0, var_y, 0.0, 0.0, 0.0, 0.0,  // position y
        0.0, 0.0, var_z, 0.0, 0.0, 0.0,  // position z
        0.0, 0.0, 0.0, 0.1, 0.0, 0.0,    // roll not calculated
        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,    // pitch not calculated
        0.0, 0.0, 0.0, 0.0, 0.0, yaw_cov // yaw
    };

    double offset_x;
    double offset_y;
    double offset_z;

    try
    {
      geometry_msgs::msg::TransformStamped tf =
          tf2_buffer_.lookupTransform(base_frame_, gnss_base_frame_, tf2::TimePointZero);

      offset_x = tf.transform.translation.x;
      offset_y = tf.transform.translation.y;
      offset_z = tf.transform.translation.z;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
          "Could not get transform %s -> %s: %s", base_frame_.c_str(), gnss_base_frame_.c_str(), ex.what());
    }

    double yaw = tf2::getYaw(q);
    // Apply offset

    pose_cov_msg.pose.pose.position.x -=
        offset_x * std::cos(yaw) - offset_y * std::sin(yaw);
    pose_cov_msg.pose.pose.position.y -=
        offset_x * std::sin(yaw) + offset_y * std::cos(yaw);
    pose_cov_msg.pose.pose.position.z -= offset_z;

    pose_cov_pub_->publish(pose_cov_msg);
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = pose_cov_msg.header.stamp;
    pose_stamped.header.frame_id = pose_cov_msg.header.frame_id;
    pose_stamped.pose = pose_cov_msg.pose.pose;
    pose_pub_->publish(pose_stamped);
    // publishTF(
    //   map_frame_, gnss_base_frame_, pose_stamped);
    // RCLCPP_INFO_THROTTLE(
    //     this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
    //     "Position published: x=%.2f, y=%.2f, z=%.2f",
    //     pose_cov_msg.pose.pose.position.x,
    //     pose_cov_msg.pose.pose.position.y,
    //     pose_cov_msg.pose.pose.position.z);
  }

  void GNSSPoser::callbackMapProjectorInfo(const MapProjectorInfo::Message::ConstSharedPtr msg)
  {
    projector_info_ = *msg;
    received_map_projector_info_ = true;
  }

  bool GNSSPoser::isFixed(const sensor_msgs::msg::NavSatStatus &nav_sat_status_msg)
  {
    return nav_sat_status_msg.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  }

  bool GNSSPoser::canGetCovariance(const sensor_msgs::msg::NavSatFix &nav_sat_fix_msg)
  {
    return nav_sat_fix_msg.position_covariance_type >
           sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }

  void GNSSPoser::publishTF(
      const std::string &frame_id, const std::string &child_frame_id,
      const geometry_msgs::msg::PoseStamped &pose_msg)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;
    transform_stamped.header.stamp = pose_msg.header.stamp;

    transform_stamped.transform.translation.x = pose_msg.pose.position.x;
    transform_stamped.transform.translation.y = pose_msg.pose.position.y;
    transform_stamped.transform.translation.z = pose_msg.pose.position.z;

    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
    transform_stamped.transform.rotation.x = tf_quaternion.x();
    transform_stamped.transform.rotation.y = tf_quaternion.y();
    transform_stamped.transform.rotation.z = tf_quaternion.z();
    transform_stamped.transform.rotation.w = tf_quaternion.w();

    tf2_broadcaster_.sendTransform(transform_stamped);
  }
} // namespace gnss_poser

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gnss_poser::GNSSPoser)
