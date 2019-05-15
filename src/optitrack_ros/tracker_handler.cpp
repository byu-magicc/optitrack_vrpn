/**
 * @file tracker_handler.cpp
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#include <optitrack_ros/tracker_handler.h>

namespace optitrack_ros
{

TrackerHandler::TrackerHandler(const std::string& name, const std::string& host, const std::shared_ptr<vrpn_Connection>& connection) :
  name_(name),
  connection_(connection),
  tracker_((name + "@" + host).c_str(), connection_.get())
{
  tracker_.register_change_handler(this, &TrackerHandler::position_callback_wrapper);
}

void TrackerHandler::position_callback_wrapper(void *userData, vrpn_TRACKERCB info)
{
  TrackerHandler *handler = reinterpret_cast<TrackerHandler*>(userData);
  handler->position_callback(info);
}

void TrackerHandler::mainloop_callback(const ros::TimerEvent& e)
{
  tracker_.mainloop();
}

void TrackerHandler::position_callback(const vrpn_TRACKERCB& info)
{
  ROS_INFO_THROTTLE(1, "%s position: (%.2f, %.2f, %.2f)", name_.c_str(), info.pos[0], info.pos[1], info.pos[2]);
}

} // namespace optitrack_ros
