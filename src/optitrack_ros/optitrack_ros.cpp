/**
 * @file optitrack_ros.cpp
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#include <optitrack_ros/optitrack_ros.h>

namespace optitrack_ros
{

OptiTrackROS::OptiTrackROS() :
  nh_private_("~")
{
  connection_ = std::shared_ptr<vrpn_Connection>(vrpn_get_connection_by_name(host_.c_str()));

  if (connection_->connected())
  {
    ROS_INFO("Connected to VRPN server at %s", host_.c_str());
  }
  else
  {
    ROS_FATAL("Unable to connect to VRPN server at %s", host_.c_str());
    ros::shutdown();
  }


  mainloop_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &OptiTrackROS::mainloop_callback, this);
  tracker_update_timer_ = nh_.createTimer(ros::Duration(1.0), &OptiTrackROS::tracker_update_callback, this);
}

void OptiTrackROS::mainloop_callback(const ros::TimerEvent& e)
{
  if (!connection_->doing_okay())
  {
      ROS_WARN_THROTTLE(1, "VRPN connection not OK");
  }
  connection_->mainloop();
}

void OptiTrackROS::tracker_update_callback(const ros::TimerEvent& e)
{
  for (int i = 0; connection_->sender_name(i) != NULL; i++)
  {
    if (sender_name_blacklist_.count(connection_->sender_name(i)) == 0)
    {
      auto result = trackers_.try_emplace(connection_->sender_name(i), connection_->sender_name(i), host_, connection_);
      if (result.second)
      {
        ROS_INFO("Added tracker %s", connection_->sender_name(i));
      }
    }
  }
}

} // namespace optitrack_ros
