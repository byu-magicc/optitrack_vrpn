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
 * @file tracker_handler.h
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#ifndef OPTITRACK_VRPN_TRACKER_HANDLER_H
#define OPTITRACK_VRPN_TRACKER_HANDLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <optitrack_vrpn/time_manager.h>

#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>

#include <memory>

namespace optitrack_vrpn
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
  TrackerHandler(const std::string& name,
                 const TrackerHandlerOptions& options,
                 const std::shared_ptr<vrpn_Connection>& connection,
                 TimeManager& time_manager);

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
  TimeManager& time_manager_;
  vrpn_Tracker_Remote tracker_;

  ros::NodeHandle nh_;
  ros::Publisher enu_pub_;
  ros::Publisher ned_pub_;
  ros::Publisher ned_att_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  ros::Timer mainloop_timer_;

  tf2::Quaternion rfu_to_flu_; //!< transforms body rotation from right-front-up to forward-left-up

  void position_callback(const vrpn_TRACKERCB& info);
  void send_transform(const geometry_msgs::PoseStamped& pose, const std::string& child_frame);
};

} // namespace optitrack_vrpn

#endif // OPTITRACK_VRPN_TRACKER_HANDLER_H
