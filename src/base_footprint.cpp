#include <humanoid_base_footprint/base_footprint.h>

BaseFootprintBroadcaster::BaseFootprintBroadcaster() : tfBuffer(ros::Duration(10.0)),  tfListener(tfBuffer)
{
    //setup tf listener and broadcaster as class members

    ros::NodeHandle n("~");
    got_support_foot = false;
    ros::Subscriber walking_support_foot_subscriber = n.subscribe("/walk_support_state", 1, &BaseFootprintBroadcaster::supportFootCallback, this, ros::TransportHints().tcpNoDelay());
    ros::Subscriber dynamic_kick_support_foot_subscriber = n.subscribe("/dynamic_kick_support_state", 1, &BaseFootprintBroadcaster::supportFootCallback, this, ros::TransportHints().tcpNoDelay());

    tf = geometry_msgs::TransformStamped();

    static tf2_ros::TransformBroadcaster br;
    ros::Rate r(30.0);
    while(ros::ok())
    {
        ros::spinOnce();
        geometry_msgs::TransformStamped tf_right,
                                        tf_left,
                                        odom,
                                        support_foot,
                                        non_support_foot,
                                        non_support_foot_in_support_foot_frame,
                                        base_footprint_in_support_foot_frame;

        try{
            tf_right = tfBuffer.lookupTransform("base_link", "r_sole", ros::Time::now(), ros::Duration(0.1));
            tf_left = tfBuffer.lookupTransform("base_link", "l_sole",  ros::Time::now(), ros::Duration(0.1));
            odom = tfBuffer.lookupTransform("base_link", "odom",  ros::Time::now(), ros::Duration(0.1));

            if(got_support_foot)
            {
                if(is_left_support)
                {
                    support_foot = tf_left;
                    non_support_foot = tf_right;
                }
                else
                {
                    support_foot = tf_right;
                    non_support_foot = tf_left;
                }
            }
            else
            {
                // check which foot is support foot (which foot is on the ground)
                if(tf_right.transform.translation.z < tf_left.transform.translation.z) {
                    support_foot = tf_right;
                    non_support_foot = tf_left;
                } else {
                    support_foot = tf_left;
                    non_support_foot = tf_right;
                }
            }
            // get the position of the non support foot in the support frame, used for computing the barycenter
            non_support_foot_in_support_foot_frame = tfBuffer.lookupTransform(support_foot.child_frame_id,
                                                                              non_support_foot.child_frame_id,
                                                                              support_foot.header.stamp,
                                                                              ros::Duration(0.1));

            geometry_msgs::TransformStamped support_to_base_link = tfBuffer.lookupTransform(support_foot.header.frame_id,
                                                                                            support_foot.child_frame_id,
                                                                                            support_foot.header.stamp);

            geometry_msgs::PoseStamped base_footprint, base_footprint_in_base_link;

            // z at ground leven (support foot height)
            base_footprint.pose.position.z = 0;
            // x and y at barycenter of feet projections on the ground
            base_footprint.pose.position.x = non_support_foot_in_support_foot_frame.transform.translation.x/2;
            base_footprint.pose.position.y = non_support_foot_in_support_foot_frame.transform.translation.y/2;


            // get yaw from base link
            double yaw;
            yaw = tf2::getYaw(odom.transform.rotation);

            // Convert tf to eigen quaternion
            Eigen::Quaterniond eigen_quat, eigen_quat_out;
            tf2::convert(odom.transform.rotation, eigen_quat);

            // Remove yaw from quaternion
            rot_conv::QuatNoEYaw(eigen_quat, eigen_quat_out);

            // Convert eigen to tf quaternion
            tf2::Quaternion tf_quat_out;

            tf2::convert(eigen_quat_out, tf_quat_out);

            // pitch and roll from support foot, yaw from base link
            tf2::Quaternion rotation;
            rotation.setRPY(0.0,0.0,yaw);

            tf2::Quaternion odom_rot;
            tf2::fromMsg(odom.transform.rotation, odom_rot);

            base_footprint.pose.orientation = tf2::toMsg(odom_rot * rotation.inverse());

            // transform the position and orientation of the base footprint into the base_link frame
            tf2::doTransform(base_footprint, base_footprint_in_base_link, support_to_base_link);

            // set the broadcasted transform to the position and orientation of the base footprint
            tf.header.stamp = base_footprint_in_base_link.header.stamp;
            tf.header.frame_id = base_footprint_in_base_link.header.frame_id;
            tf.child_frame_id = "base_footprint";
            tf.transform.translation.x = base_footprint_in_base_link.pose.position.x;
            tf.transform.translation.y = base_footprint_in_base_link.pose.position.y;
            tf.transform.translation.z = base_footprint_in_base_link.pose.position.z;
            tf.transform.rotation = base_footprint.pose.orientation;
            br.sendTransform(tf);

        } catch(...){
            //ROS_WARN_THROTTLE(2, "Can not publish base_footprint, check your tf tree");
            continue;
        }

        r.sleep();
    }
}

void BaseFootprintBroadcaster::supportFootCallback(const std_msgs::Char msg)
{
    got_support_foot = true;
    is_left_support = (msg.data == 'l');
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_footprint");

    BaseFootprintBroadcaster b;
}

