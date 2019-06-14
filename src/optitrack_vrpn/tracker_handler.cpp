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
 * @file tracker_handler.cpp
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#include <optitrack_vrpn/tracker_handler.h>

#include <geometry_msgs/TransformStamped.h>

#include <cmath>

namespace optitrack_vrpn
{

TrackerHandler::TrackerHandler(const std::string& name,
                               const TrackerHandlerOptions& options,
                               const std::shared_ptr<vrpn_Connection>& connection,
                               TimeManager& time_manager) :
  name_(name),
  options_(options),
  tf_child_frame_(name),
  tf_child_frame_ned_(name + "_ned"),
  connection_(connection),
  time_manager_(time_manager),
  tracker_((name + "@" + options.host).c_str(), connection_.get())
{
  rfu_to_flu_.setRPY(0.0, 0.0, M_PI_2); // rotate pi/2 (90deg) about z-axis

  enu_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(name_ + "_enu", 1);
  ned_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(name_ + "_ned", 1);
  ned_att_pub_ = nh_.advertise<geometry_msgs::Quaternion>(name_ + "_att", 1);

  tracker_.register_change_handler(this, &TrackerHandler::position_callback_wrapper);
}

void TrackerHandler::position_callback_wrapper(void *userData, vrpn_TRACKERCB info)
{
  TrackerHandler *handler = reinterpret_cast<TrackerHandler*>(userData);
  handler->position_callback(info);
}

void TrackerHandler::position_callback(const vrpn_TRACKERCB& info)
{
  ros::Time stamp = time_manager_.resolve_timestamp(info.msg_time);

  // axis mappings for ENU:
  //   East  (X): OptiTrack Z
  //   North (Y): OptiTrack X
  //   Up    (Z): OptiTrack Y
  geometry_msgs::PoseStamped enu_msg;
  enu_msg.header.stamp = stamp;
  enu_msg.header.frame_id = options_.frame;

  enu_msg.pose.position.x =  info.pos[VRPNIndex::Z];
  enu_msg.pose.position.y =  info.pos[VRPNIndex::X];
  enu_msg.pose.position.z =  info.pos[VRPNIndex::Y];

  // first get rotation to right-left-up body frame, then rotate to forward-left-down
  tf2::Quaternion enu_to_rlu(info.quat[VRPNIndex::Z], // x
                             info.quat[VRPNIndex::X], // y
                             info.quat[VRPNIndex::Y], // z
                             info.quat[VRPNIndex::W]); // w
  tf2::Quaternion quat = enu_to_rlu * rfu_to_flu_;

  enu_msg.pose.orientation.x =  quat.x();
  enu_msg.pose.orientation.y =  quat.y();
  enu_msg.pose.orientation.z =  quat.z();
  enu_msg.pose.orientation.w =  quat.w();
  enu_pub_.publish(enu_msg);
  send_transform(enu_msg, tf_child_frame_);

  // axis mappings for NED:
  //   North (X): OptiTrack X
  //   East  (Y): OptiTrack Z
  //   Down  (Z): OptiTrack -Y
  geometry_msgs::PoseStamped ned_msg;
  ned_msg.header.stamp = stamp;
  ned_msg.header.frame_id = options_.ned_frame;
  ned_msg.pose.position.x =  info.pos[VRPNIndex::X];
  ned_msg.pose.position.y =  info.pos[VRPNIndex::Z];
  ned_msg.pose.position.z = -info.pos[VRPNIndex::Y];
  ned_msg.pose.orientation.x =  info.quat[VRPNIndex::X];
  ned_msg.pose.orientation.y =  info.quat[VRPNIndex::Z];
  ned_msg.pose.orientation.z = -info.quat[VRPNIndex::Y];
  ned_msg.pose.orientation.w =  info.quat[VRPNIndex::W];
  ned_pub_.publish(ned_msg);
  send_transform(ned_msg, tf_child_frame_ned_);

  ned_att_pub_.publish(ned_msg.pose.orientation);
}

void TrackerHandler::send_transform(const geometry_msgs::PoseStamped &pose, const std::string &child_frame)
{
  geometry_msgs::TransformStamped tf;

  tf.header.stamp = pose.header.stamp;
  tf.header.frame_id = pose.header.frame_id;
  tf.child_frame_id = child_frame;

  tf.transform.translation.x = pose.pose.position.x;
  tf.transform.translation.y = pose.pose.position.y;
  tf.transform.translation.z = pose.pose.position.z;

  tf.transform.rotation.x = pose.pose.orientation.x;
  tf.transform.rotation.y = pose.pose.orientation.y;
  tf.transform.rotation.z = pose.pose.orientation.z;
  tf.transform.rotation.w = pose.pose.orientation.w;

  tf_broadcaster_.sendTransform(tf);
}

} // namespace optitrack_vrpn
