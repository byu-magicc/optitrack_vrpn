/*
 * Copyright 2019 Daniel Koch, BYU MAGICC Lab
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file optitrack_vrpn.cpp
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#include <optitrack_vrpn/optitrack_vrpn.h>
#include <geometry_msgs/TransformStamped.h>

namespace optitrack_vrpn
{

OptiTrackVRPN::OptiTrackVRPN() :
  nh_private_("~")
{
  nh_private_.param<std::string>("host", host_, "192.168.1.186");
  nh_private_.param<int>("update_rate", update_rate_, 500);
  nh_private_.param<std::string>("frame", frame_, "optitrack");
  ned_frame_ = frame_ + "_ned";

  options_.host = host_;
  options_.frame = frame_;
  options_.ned_frame = ned_frame_;

  int offset_samples;
  nh_private_.param<int>("offset_samples", offset_samples, 100);
  time_manager_.set_num_samples(offset_samples);

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

  mainloop_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &OptiTrackVRPN::mainloop_callback, this);
  tracker_update_timer_ = nh_.createTimer(ros::Duration(1.0), &OptiTrackVRPN::tracker_update_callback, this);
}

void OptiTrackVRPN::mainloop_callback(const ros::TimerEvent& e)
{
  if (!connection_->doing_okay())
  {
    ROS_WARN_THROTTLE(1, "VRPN connection not OK");
  }
  connection_->mainloop();
}

void OptiTrackVRPN::tracker_update_callback(const ros::TimerEvent& e)
{
  for (int i = 0; connection_->sender_name(i) != NULL; i++)
  {
    if (sender_name_blacklist_.count(connection_->sender_name(i)) == 0
          && trackers_.count(connection_->sender_name(i)) == 0)
    {
      trackers_.emplace(std::piecewise_construct,
                        std::forward_as_tuple(connection_->sender_name(i)),
                        std::forward_as_tuple(connection_->sender_name(i), options_, connection_, time_manager_));
      ROS_INFO("Added tracker for rigid body \"%s\"", connection_->sender_name(i));
    }
  }
}

void OptiTrackVRPN::publish_enu_to_ned_transform()
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

} // namespace optitrack_vrpn
