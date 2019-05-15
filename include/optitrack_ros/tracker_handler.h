/**
 * @file tracker_handler.h
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#ifndef OPTITRACK_ROS_TRACKER_HANDLER_H
#define OPTITRACK_ROS_TRACKER_HANDLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>

#include <memory>

namespace optitrack_ros
{

/**
 * @brief Options for TrackerHandler class
 */
struct TrackerHandlerOptions
{
  std::string host;
  std::string frame;
  std::string ned_frame;
};

/**
 * @brief Handles all callbacks for a single tracker (one tracker per rigid body)
 */
class TrackerHandler
{
public:
  TrackerHandler(const std::string& name, const std::shared_ptr<vrpn_Connection>& connection, const TrackerHandlerOptions& options);

  static void VRPN_CALLBACK position_callback_wrapper(void *userData, vrpn_TRACKERCB info);

private:
  enum VRPNIndex
  {
    X = 0,
    Y = 1,
    Z = 2,
    W = 3
  };

  std::string name_;
  const TrackerHandlerOptions& options_;

  std::string tf_child_frame_;
  std::string tf_child_frame_ned_;

  std::shared_ptr<vrpn_Connection> connection_;
  vrpn_Tracker_Remote tracker_;

  ros::NodeHandle nh_;
  ros::Publisher enu_pub_;
  ros::Publisher ned_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  ros::Timer mainloop_timer_;

  tf2::Quaternion rfu_to_flu_; //!< transforms body rotation from right-front-up to forward-left-up

  void position_callback(const vrpn_TRACKERCB& info);
  void send_transform(const geometry_msgs::PoseStamped& pose, const std::string& child_frame);
};

} // namespace optitrack_ros

#endif // OPTITRACK_ROS_TRACKER_HANDLER_H
