/**
 * @file tracker_handler.cpp
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#include <optitrack_ros/tracker_handler.h>

#include <geometry_msgs/PoseStamped.h>

namespace optitrack_ros
{

TrackerHandler::TrackerHandler(const std::string& name, const std::string& host, const std::shared_ptr<vrpn_Connection>& connection) :
  name_(name),
  connection_(connection),
  tracker_((name + "@" + host).c_str(), connection_.get())
{
  tracker_.register_change_handler(this, &TrackerHandler::position_callback_wrapper);

  enu_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(name_ + "_enu", 1);
  ned_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(name_ + "_ned", 1);
}

void TrackerHandler::position_callback_wrapper(void *userData, vrpn_TRACKERCB info)
{
  TrackerHandler *handler = reinterpret_cast<TrackerHandler*>(userData);
  handler->position_callback(info);
}

void TrackerHandler::position_callback(const vrpn_TRACKERCB& info)
{
  //! @todo intelligent time stamp mechanism
  ros::Time stamp = ros::Time::now();

  geometry_msgs::PoseStamped enu_msg;
  enu_msg.header.stamp = stamp;
  enu_msg.pose.position.x =  info.pos[VRPNIndex::Z];
  enu_msg.pose.position.y =  info.pos[VRPNIndex::X];
  enu_msg.pose.position.z =  info.pos[VRPNIndex::Y];
  enu_msg.pose.orientation.x =  info.pos[VRPNIndex::Z];
  enu_msg.pose.orientation.y =  info.pos[VRPNIndex::X];
  enu_msg.pose.orientation.z =  info.pos[VRPNIndex::Y];
  enu_msg.pose.orientation.w =  info.pos[VRPNIndex::W];
  enu_pub_.publish(enu_msg);

  geometry_msgs::PoseStamped ned_msg;
  ned_msg.header.stamp = stamp;
  ned_msg.pose.position.x =  info.pos[VRPNIndex::X];
  ned_msg.pose.position.y =  info.pos[VRPNIndex::Z];
  ned_msg.pose.position.z = -info.pos[VRPNIndex::Y];
  ned_msg.pose.orientation.x =  info.pos[VRPNIndex::X];
  ned_msg.pose.orientation.y =  info.pos[VRPNIndex::Z];
  ned_msg.pose.orientation.z = -info.pos[VRPNIndex::Y];
  ned_msg.pose.orientation.w =  info.pos[VRPNIndex::W];
  ned_pub_.publish(ned_msg);
}

} // namespace optitrack_ros
