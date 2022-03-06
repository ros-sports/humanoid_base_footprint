// Copyright 2022 Hamburg Bit-Bots
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

#include <rot_conv/rot_conv.h>
#include <Eigen/Geometry>
#include <functional>
#include <humanoid_base_footprint/base_footprint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// This has to be included after #include <tf2_geometry_msgs/tf2_geometyr_msgs.hpp>, due to reasons
// explained in https://github.com/ros2/geometry2/pull/485
// "tf2/utils.h" is used instead of <tf2/utils.h> to prevent ament_cpplint from complaining about
// including a C system header after C++ system headers.
#include "tf2/utils.h"

using std::placeholders::_1;

namespace humanoid_base_footprint
{

BaseFootprintBroadcaster::BaseFootprintBroadcaster(const rclcpp::NodeOptions &)
: Node("base_footprint"),
  tfBuffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
  tfListener_(std::make_shared<tf2_ros::TransformListener>(*tfBuffer_))
{
  // setup tf listener and broadcaster as class members
  this->declare_parameter<std::string>("base_link_frame", "base_link");
  this->get_parameter("base_link_frame", base_link_frame_);
  this->declare_parameter<std::string>("base_footprint_frame", "base_footprint");
  this->get_parameter("base_footprint_frame", base_footprint_frame_);
  this->declare_parameter<std::string>("r_sole_frame", "r_sole");
  this->get_parameter("r_sole_frame", r_sole_frame_);
  this->declare_parameter<std::string>("l_sole_frame", "l_sole");
  this->get_parameter("l_sole_frame", l_sole_frame_);
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->get_parameter("odom_frame", odom_frame_);
  got_support_foot_ = false;

  this->declare_parameter<std::vector<std::string>>("support_state_topics", {"walk_support_state"});
  std::vector<std::string> support_state_topics;
  this->get_parameter("support_state_topics", support_state_topics);
  for (auto topic : support_state_topics) {
    this->create_subscription<biped_interfaces::msg::Phase>(
      topic,
      1,
      std::bind(
        &BaseFootprintBroadcaster::supportFootCallback,
        this,
        _1));
  }
}

void BaseFootprintBroadcaster::loop()
{
  tf_ = geometry_msgs::msg::TransformStamped();

  static std::unique_ptr<tf2_ros::TransformBroadcaster> br =
    std::make_unique<tf2_ros::TransformBroadcaster>(this);
  auto node_pointer = this->shared_from_this();

  rclcpp::Time last_published_time;
  while (rclcpp::ok()) {
    rclcpp::Time startTime = this->get_clock()->now();
    rclcpp::spin_some(node_pointer);
    geometry_msgs::msg::TransformStamped tf_right,
      tf_left,
      odom,
      support_foot,
      non_support_foot,
      non_support_foot_in_support_foot_frame,
      base_footprint_in_support_foot_frame;

    try {
      tf_right = tfBuffer_
        ->lookupTransform(
        base_link_frame_,
        r_sole_frame_,
        this->now(),
        rclcpp::Duration::from_nanoseconds(1e9 * 0.1));
      tf_left = tfBuffer_
        ->lookupTransform(
        base_link_frame_,
        l_sole_frame_,
        this->now(),
        rclcpp::Duration::from_nanoseconds(1e9 * 0.1));
      odom = tfBuffer_
        ->lookupTransform(
        base_link_frame_, odom_frame_,
        this->now(), rclcpp::Duration::from_nanoseconds(1e9 * 0.1));

      if (got_support_foot_) {
        if (is_left_support_) {
          support_foot = tf_left;
          non_support_foot = tf_right;
        } else {
          support_foot = tf_right;
          non_support_foot = tf_left;
        }
      } else {
        // check which foot is support foot (which foot is on the ground)
        if (tf_right.transform.translation.z < tf_left.transform.translation.z) {
          support_foot = tf_right;
          non_support_foot = tf_left;
        } else {
          support_foot = tf_left;
          non_support_foot = tf_right;
        }
      }
      // get the position of the non-support foot in the support frame, used for computing the
      // barycenter
      non_support_foot_in_support_foot_frame = tfBuffer_->lookupTransform(
        support_foot.child_frame_id,
        non_support_foot.child_frame_id,
        support_foot.header.stamp,
        rclcpp::Duration::from_nanoseconds(
          1e9 * 0.1));

      geometry_msgs::msg::TransformStamped
        support_to_base_link = tfBuffer_->lookupTransform(
        support_foot.header.frame_id,
        support_foot.child_frame_id,
        support_foot.header.stamp);

      geometry_msgs::msg::PoseStamped base_footprint, base_footprint_in_base_link;

      // z at ground leven (support foot height)
      base_footprint.pose.position.z = 0;
      // x and y at barycenter of feet projections on the ground
      base_footprint.pose.position.x =
        non_support_foot_in_support_foot_frame.transform.translation.x / 2;
      base_footprint.pose.position.y =
        non_support_foot_in_support_foot_frame.transform.translation.y / 2;


      // get yaw from base link
      double yaw;
      yaw = tf2::getYaw(odom.transform.rotation);

      // Convert tf to eigen quaternion
      Eigen::Quaterniond eigen_quat
        (odom.transform.rotation.w, odom.transform.rotation.x, odom.transform.rotation.y,
        odom.transform.rotation.z);
      // can't use this out of some reasons tf2::convert(odom.transform.rotation, eigen_quat);

      // Remove yaw from quaternion
      Eigen::Quaterniond eigen_quat_out;
      rot_conv::QuatNoEYaw(eigen_quat, eigen_quat_out);

      // Convert eigen to tf quaternion
      tf2::Quaternion tf_quat_out;
      tf2::convert(eigen_quat_out, tf_quat_out);

      // pitch and roll from support foot, yaw from base link
      tf2::Quaternion rotation;
      rotation.setRPY(0.0, 0.0, yaw);

      tf2::Quaternion odom_rot;
      tf2::fromMsg(odom.transform.rotation, odom_rot);

      base_footprint.pose.orientation = tf2::toMsg(odom_rot * rotation.inverse());

      // transform the position and orientation of the base footprint into the base_link frame
      tf2::doTransform(base_footprint, base_footprint_in_base_link, support_to_base_link);

      // set the broadcasted transform to the position and orientation of the base footprint
      tf_.header.stamp = base_footprint_in_base_link.header.stamp;
      tf_.header.frame_id = base_footprint_in_base_link.header.frame_id;
      tf_.child_frame_id = base_footprint_frame_;
      tf_.transform.translation.x = base_footprint_in_base_link.pose.position.x;
      tf_.transform.translation.y = base_footprint_in_base_link.pose.position.y;
      tf_.transform.translation.z = base_footprint_in_base_link.pose.position.z;
      tf_.transform.rotation = base_footprint.pose.orientation;
      if (tf_.header.stamp != last_published_time) {
        br->sendTransform(tf_);
        last_published_time = tf_.header.stamp;
      }
    } catch (...) {
      // RCLCPP_WARN_THROTTLE(
      //   this->get_logger(), *this->get_clock(), 2,
      //   "Can not->publish base_footprint, check your tf tree");
      continue;
    }

    this->get_clock()->sleep_until(
      startTime + rclcpp::Duration::from_nanoseconds(1e9 / 30));
  }
}

void BaseFootprintBroadcaster::supportFootCallback(const biped_interfaces::msg::Phase msg)
{
  got_support_foot_ = true;
  is_left_support_ = (msg.phase == biped_interfaces::msg::Phase::LEFT_STANCE);
}

}  // namespace humanoid_base_footprint

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(humanoid_base_footprint::BaseFootprintBroadcaster)
