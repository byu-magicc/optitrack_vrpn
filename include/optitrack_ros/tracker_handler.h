/**
 * @file tracker_handler.h
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#ifndef OPTITRACK_ROS_TRACKER_HANDLER_H
#define OPTITRACK_ROS_TRACKER_HANDLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>

#include <memory>

namespace optitrack_ros
{

/**
 * @brief Handles all callbacks for a single tracker (one tracker per rigid body)
 */
class TrackerHandler
{
public:
  TrackerHandler(const std::string& name, const std::string& host, const std::shared_ptr<vrpn_Connection>& connection);

  static void position_callback_wrapper(void *userData, vrpn_TRACKERCB info);

private:
  std::string name_;

  std::shared_ptr<vrpn_Connection> connection_;
  vrpn_Tracker_Remote tracker_;

  ros::NodeHandle nh_;
  ros::Publisher ned_pub_;
  ros::Publisher enu_pub_;

  ros::Timer mainloop_timer_;

  void mainloop_callback(const ros::TimerEvent& e);

  void position_callback(const vrpn_TRACKERCB& info);
};

} // namespace optitrack_ros

#endif // OPTITRACK_ROS_TRACKER_HANDLER_H
