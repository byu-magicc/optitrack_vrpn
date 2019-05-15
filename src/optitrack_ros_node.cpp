/**
 * @file optitrack_ros_node.cpp
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#include <ros/ros.h>
#include <optitrack_ros/optitrack_ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "optitrack_ros_node");

  optitrack_ros::OptiTrackROS handler;
  ros::spin();

  return 0;
}