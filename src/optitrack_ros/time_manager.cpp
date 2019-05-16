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
 * @file time_manager.cpp
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#include <optitrack_ros/time_manager.h>

namespace optitrack_ros
{

TimeManager::TimeManager() :
  min_offset_(0.0),
  count_(0)
{}

ros::Time TimeManager::resolve_timestamp(const timeval& stamp)
{
  if (count_ >= num_samples_)
  {
    return timeval_to_ros_time(stamp) + offset_;
  }
  else
  {
    double offset = (ros::Time::now() - timeval_to_ros_time(stamp)).toSec();

    if (count_ == 0 || offset < min_offset_)
    {
      min_offset_ = offset;
    }
    count_++;

    if (count_ == num_samples_)
    {
      offset_ = ros::Duration(min_offset_);
    }

    return ros::Time::now();
  }
}

void TimeManager::set_num_samples(size_t num_samples)
{
  num_samples_ = num_samples;
}

ros::Time TimeManager::timeval_to_ros_time(const timeval& stamp)
{
  return ros::Time(stamp.tv_sec, stamp.tv_usec*1000);
}

} // namespace optitrack_ros
