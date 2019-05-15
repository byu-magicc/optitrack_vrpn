/**
 * @file optitrack_ros.cpp
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#include <optitrack_ros/optitrack_ros.h>
#include <geometry_msgs/TransformStamped.h>

namespace optitrack_ros
{

OptiTrackROS::OptiTrackROS() :
  nh_private_("~")
{
  nh_private_.param<std::string>("host", host_, "192.168.1.186");
  nh_private_.param<int>("update_rate", update_rate_, 500);
  nh_private_.param<std::string>("frame", frame_, "optitrack");
  ned_frame_ = frame_ + "_ned";

  options_.host = host_;
  options_.frame = frame_;
  options_.ned_frame = ned_frame_;

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

  publish_enu_to_ned_transform();

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
      auto result = trackers_.try_emplace(connection_->sender_name(i), connection_->sender_name(i), connection_, options_);
      if (result.second)
      {
        ROS_INFO("Added tracker %s", connection_->sender_name(i));
      }
    }
  }
}

void OptiTrackROS::publish_enu_to_ned_transform()
{
  geometry_msgs::TransformStamped enu_to_ned;

  enu_to_ned.header.stamp = ros::Time::now();
  enu_to_ned.header.frame_id = frame_;
  enu_to_ned.child_frame_id = ned_frame_;

  enu_to_ned.transform.translation.x = 0.0;
  enu_to_ned.transform.translation.y = 0.0;
  enu_to_ned.transform.translation.z = 0.0;

  enu_to_ned.transform.rotation.x = std::sqrt(0.5);
  enu_to_ned.transform.rotation.y = std::sqrt(0.5);
  enu_to_ned.transform.rotation.z = 0;
  enu_to_ned.transform.rotation.w = 0;

  static_tf_broadcaster_.sendTransform(enu_to_ned);
}

} // namespace optitrack_ros
