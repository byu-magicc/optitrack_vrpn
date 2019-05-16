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
 * @file optitrack_vrpn.h
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#ifndef OPTITRACK_VRPN_OPTITRACK_VRPN_H
#define OPTITRACK_VRPN_OPTITRACK_VRPN_H

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <vrpn_Connection.h>

#include <memory>
#include <map>
#include <set>

#include <optitrack_vrpn/time_manager.h>
#include <optitrack_vrpn/tracker_handler.h>

namespace optitrack_vrpn
{

/**
 * @brief Manages the connection to Motive and initializes tracker handlers
 */
class OptiTrackVRPN
{
public:
  OptiTrackVRPN();

private:
  std::set<std::string> sender_name_blacklist_ = std::set<std::string>({"VRPN Control"});

  std::string host_;
  int update_rate_;

  std::string frame_;
  std::string ned_frame_;

  TrackerHandlerOptions options_;

  std::shared_ptr<vrpn_Connection> connection_;
  std::map<std::string,TrackerHandler> trackers_;
  TimeManager time_manager_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer mainloop_timer_;
  ros::Timer tracker_update_timer_;

  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  void mainloop_callback(const ros::TimerEvent& e);
  void tracker_update_callback(const ros::TimerEvent& e);

  void publish_enu_to_ned_transform();
};

} // namespace optitrack_vrpn

#endif // OPTITRACK_VRPN_OPTITRACK_VRPN_H
