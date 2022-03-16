# humanoid_base_footprint

[![Build and Test (foxy)](../../actions/workflows/build_and_test_foxy.yaml/badge.svg)](../../actions/workflows/build_and_test_foxy.yaml)
[![Build and Test (galactic)](../../actions/workflows/build_and_test_galactic.yaml/badge.svg)](../../actions/workflows/build_and_test_galactic.yaml)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg)](../../actions/workflows/build_and_test_rolling.yaml)

This ROS2 package includes a node which provides the base footprint frame for any humanoid robot following [REP - 120](https://www.ros.org/reps/rep-0120.html).

## Installation

Only source installation is available currently. Run the following in your ROS workspace:

```
git clone https://github.com/ros-sports/humanoid_base_footprint.git src/humanoid_base_footprint
vcs import src < src/humanoid_base_footprint/dependencies.repos --recursive
colcon build
```

## Description

Definition of the base footprint frame (from [here](https://www.ros.org/reps/rep-0120.html)):

>The base_footprint is the representation of the robot position on the floor. The floor is usually the level where the supporting leg rests, i.e. z = min(l_sole_z, r_sole_z) where l_sole_z and r_sole_z are the left and right sole height respecitvely. The translation component of the frame should be the barycenter of the feet projections on the floor. With respect to the odom frame, the roll and pitch angles should be zero and the yaw angle should correspond to the base_link yaw angle.
>
>Rationale: base_footprint provides a fairly stable 2D planar representation of the humanoid even while walking and swaying with the base_link.

The node listens to [Phase]() msgs that define the current stance of the robot. By default, it will listen on the `walk_support_state` topic, but you can define a list of topics on the `support_state_topics` parameter.  

If the links of your robot have non-standard names you can set the following parameters:
`base_link_frame`, `r_sole_frame`, `l_sole_frame`, `l_sole_frame`, `odom_frame`
 

The resulting frame will be published as `base_footprint` but the name can be changed by setting the `base_footprint_frame` parameter.
