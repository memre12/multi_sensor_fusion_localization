// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef GNSS_POSER__GNSS_POSER_CORE_HPP_
#define GNSS_POSER__GNSS_POSER_CORE_HPP_

#include <component_interface_specs/map.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/circular_buffer.hpp>

#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace gnss_poser
{
class GNSSPoser : public rclcpp::Node
{
public:
  explicit GNSSPoser(const rclcpp::NodeOptions & node_options);

private:
  using MapProjectorInfo = map_interface::MapProjectorInfo;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, sensor_msgs::msg::NavSatFix>;

  void syncCallback(
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_fix_msg_ptr_1,
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_fix_msg_ptr_2);
  void callbackMapProjectorInfo(const MapProjectorInfo::Message::ConstSharedPtr msg);

  bool isFixed(const sensor_msgs::msg::NavSatStatus & nav_sat_status_msg);
  bool canGetCovariance(const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg);

  void publishTF(
    const std::string & frame_id, const std::string & child_frame_id,
    const geometry_msgs::msg::PoseStamped & pose_msg);

  tf2::BufferCore tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_broadcaster_;

  component_interface_utils::Subscription<MapProjectorInfo>::SharedPtr sub_map_projector_info_;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> sub_navsat1_;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> sub_navsat2_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  MapProjectorInfo::Message projector_info_;
  const std::string base_frame_;
  const std::string gnss_base_frame_;
  const std::string map_frame_;
  bool received_map_projector_info_ = false;
  std::string first_navsat_msg;
  std::string second_navsat_msg;
  std::string publish_topic_;
  std::string publish_topic_pose_stamped_;
};
}  // namespace gnss_poser

#endif  // GNSS_POSER__GNSS_POSER_CORE_HPP_
