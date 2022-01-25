#include <humanoid_base_footprint/base_footprint.h>

BaseFootprintBroadcaster::BaseFootprintBroadcaster()
    : Node("base_footprint"),
      tfBuffer(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
      tfListener(std::make_shared<tf2_ros::TransformListener>(*tfBuffer)) {
  //setup tf listener and broadcaster as class members



  std::string base_link_frame, base_footprint_frame, r_sole_frame, l_sole_frame, odom_frame;
  this->declare_parameter<std::string>("base_link_frame", "base_link");
  this->get_parameter("base_link_frame", base_link_frame);
  this->declare_parameter<std::string>("base_footprint_frame", "base_footprint");
  this->get_parameter("base_footprint_frame", base_footprint_frame);
  this->declare_parameter<std::string>("r_sole_frame", "r_sole");
  this->get_parameter("r_sole_frame", r_sole_frame);
  this->declare_parameter<std::string>("l_sole_frame", "l_sole");
  this->get_parameter("l_sole_frame", l_sole_frame);
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->get_parameter("odom_frame", odom_frame);
  got_support_foot = false;
  rclcpp::Subscription<bitbots_msgs::msg::SupportState>::SharedPtr walking_support_foot_subscriber =
      this->create_subscription<bitbots_msgs::msg::SupportState>("walk_support_state",
                                                                 1,
                                                                 std::bind(&BaseFootprintBroadcaster::supportFootCallback,
                                                                           this,
                                                                           _1));
  rclcpp::Subscription<bitbots_msgs::msg::SupportState>::SharedPtr dynamic_kick_support_foot_subscriber =
      this->create_subscription<bitbots_msgs::msg::SupportState>("dynamic_kick_support_state",
                                                                 1,
                                                                 std::bind(&BaseFootprintBroadcaster::supportFootCallback,
                                                                           this,
                                                                           _1));

  tf = geometry_msgs::msg::TransformStamped();

  static std::unique_ptr<tf2_ros::TransformBroadcaster> br;
  rclcpp::Rate r(30.0);
  rclcpp::Time last_published_time;
  while (rclcpp::ok()) {
    rclcpp::spin_some(std::make_shared<BaseFootprintBroadcaster>());
    geometry_msgs::msg::TransformStamped tf_right,
        tf_left,
        odom,
        support_foot,
        non_support_foot,
        non_support_foot_in_support_foot_frame,
        base_footprint_in_support_foot_frame;

    try {
      tf_right = tfBuffer->lookupTransform(base_link_frame, r_sole_frame, this->now(), rclcpp::Duration::from_nanoseconds(1e9 * 0.1));
      tf_left = tfBuffer->lookupTransform(base_link_frame, l_sole_frame, this->now(), rclcpp::Duration::from_nanoseconds(1e9 * 0.1));
      odom = tfBuffer->lookupTransform(base_link_frame, odom_frame, this->now(), rclcpp::Duration::from_nanoseconds(1e9 * 0.1));

      if (got_support_foot) {
        if (is_left_support) {
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
      // get the position of the non support foot in the support frame, used for computing the barycenter
      non_support_foot_in_support_foot_frame = tfBuffer->lookupTransform(support_foot.child_frame_id,
                                                                        non_support_foot.child_frame_id,
                                                                        support_foot.header.stamp,
                                                                        rclcpp::Duration::from_nanoseconds(1e9 * 0.1));

      geometry_msgs::msg::TransformStamped support_to_base_link = tfBuffer->lookupTransform(support_foot.header.frame_id,
                                                                                           support_foot.child_frame_id,
                                                                                           support_foot.header.stamp);

      geometry_msgs::msg::PoseStamped base_footprint, base_footprint_in_base_link;

      // z at ground leven (support foot height)
      base_footprint.pose.position.z = 0;
      // x and y at barycenter of feet projections on the ground
      base_footprint.pose.position.x = non_support_foot_in_support_foot_frame.transform.translation.x / 2;
      base_footprint.pose.position.y = non_support_foot_in_support_foot_frame.transform.translation.y / 2;


      // get yaw from base link
      double yaw;
      yaw = tf2::getYaw(odom.transform.rotation);

      // Convert tf to eigen quaternion
      Eigen::Quaterniond eigen_quat(odom.transform.rotation.w, odom.transform.rotation.x, odom.transform.rotation.y, odom.transform.rotation.z);
      //cant use this out of some reasons tf2::convert(odom.transform.rotation, eigen_quat);

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
      tf.header.stamp = base_footprint_in_base_link.header.stamp;
      tf.header.frame_id = base_footprint_in_base_link.header.frame_id;
      tf.child_frame_id = base_footprint_frame;
      tf.transform.translation.x = base_footprint_in_base_link.pose.position.x;
      tf.transform.translation.y = base_footprint_in_base_link.pose.position.y;
      tf.transform.translation.z = base_footprint_in_base_link.pose.position.z;
      tf.transform.rotation = base_footprint.pose.orientation;
      if (tf.header.stamp != last_published_time) {
        br->sendTransform(tf);
        last_published_time = tf.header.stamp;
      }

    } catch (...) {
      //RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2, "Can not->publish base_footprint, check your tf tree");
      continue;
    }

    r.sleep();
  }
}

void BaseFootprintBroadcaster::supportFootCallback(const bitbots_msgs::msg::SupportState msg) {
  got_support_foot = true;
  is_left_support = (msg.state == bitbots_msgs::msg::SupportState::LEFT);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  BaseFootprintBroadcaster b;
}

