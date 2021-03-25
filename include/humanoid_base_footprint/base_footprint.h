/*
taken from REP120

base_footprint

The base_footprint is the representation of the robot position on the floor.
The floor is usually the level where the supporting leg rests,
i.e. z = min(l_sole_z, r_sole_z) where l_sole_z and r_sole_z are the left and right sole height respecitvely.
The translation component of the frame should be the barycenter of the feet projections on the floor.
With respect to the odom frame, the roll and pitch angles should be zero and the yaw angle should correspond to the base_link yaw angle.

Rationale: base_footprint provides a fairly stable 2D planar representation of the humanoid even while walking and swaying with the base_link.
*/

#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rot_conv/rot_conv.h>
#include <tf2/utils.h>
#include <Eigen/Geometry>
#include <bitbots_msgs/SupportState.h>


class BaseFootprintBroadcaster
{
public:
    BaseFootprintBroadcaster();
private:
    geometry_msgs::TransformStamped tf;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    bool is_left_support, got_support_foot;
    void supportFootCallback(const bitbots_msgs::SupportState msg);
};
