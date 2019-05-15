/**
 * @file optitrack_ros.h
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#ifndef OPTITRACK_ROS_OPTITRACK_ROS_H
#define OPTITRACK_ROS_OPTITRACK_ROS_H

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <vrpn_Connection.h>

#include <memory>
#include <map>
#include <set>

#include <optitrack_ros/tracker_handler.h>

namespace optitrack_ros
{

/**
 * @brief Manages the connection to Motive and initializes tracker handlers
 */
class OptiTrackROS
{
public:
  OptiTrackROS();

private:
  std::set<std::string> sender_name_blacklist_ = std::set<std::string>({"VRPN Control"});

  std::string host_;
  int update_rate_;

  std::string frame_;
  std::string ned_frame_;

  TrackerHandlerOptions options_;

  std::shared_ptr<vrpn_Connection> connection_;
  std::map<std::string,TrackerHandler> trackers_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer mainloop_timer_;
  ros::Timer tracker_update_timer_;

  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  void mainloop_callback(const ros::TimerEvent& e);
  void tracker_update_callback(const ros::TimerEvent& e);

  void publish_enu_to_ned_transform();
};

} // namespace optitrack_ros

#endif // OPTITRACK_ROS_OPTITRACK_ROS_H
