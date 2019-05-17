# optitrack_vrpn

[![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__optitrack_vrpn__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__optitrack_vrpn__ubuntu_bionic_amd64)

This ROS package provides a [VRPN](https://github.com/vrpn/vrpn/wiki) client for OptiTrack's motion capture system and Motive software. Refer to the [wiki page](http://wiki.ros.org/optitrack_vrpn) for usage details.

Much of the functionality in this package is very similar to other VPRN clients such as [vrpn_client_ros](http://wiki.ros.org/vrpn_client_ros), but functionality has been added that addresses some of the issues specific to interfacing with an OptiTrack system. These issues are discussed below.

## Coordinate Frames

The default OptiTrack coordinate system has the x-axis forward, y-axis up, and z-axis to the right. This package converts to the ROS standard ENU frame, as well as the NED frame for aerospace applications.

To keep the ENU and NED frames consistent, the OptiTrack x-axis is assumed to be "North." This means that a translation along that axis will correspond to to a translation along the NED x-axis, but along the ENU y-axis. For both ENU and NED, the body x-axis goes out the front of the vehicle. These conventions are consistent with [REP 103](https://www.ros.org/reps/rep-0103.html).

## Timing

The OptiTrack system timestamps its motion capture frames very precisely. However, these timestamps are defined relative to when the Motive software was started, and do not use the system time.

On the client side, there are two options for timestamping the data. One option is to stamp each message with the current system time on the client computer when the message arrives. This method is affected by network latency, and will also result in jitter in the timestamp deltas due to the polling mechanism of the VRPN library and changing network latencies. The other option is to use the Motive timestamps, but these timestamps appear to be many years in the past compared to the system time.

This package uses a hybrid approach for timestamping data. For the first N (default N=100) messages, the Motive timestamp is compared with the client system-time time-of-arrival stamp. The minimum of the differences between these timestamp sources (corresponding to the lowest-latency message) is then used to define an offset between the client system time and the Motive time. All following messages are then timestamped by adding this constant offset to the Motive timestamps.

This method solves the problem of jitter in the timestamp deltas. It does not, however, correct for the network latency. If this is important for your application and you are not able to sufficiently reduce the network latency, one potential solution is to run this node on the same Windows machine as Motive, using the Linux subsystem for Windows. You can then set up an [NTP](https://en.wikipedia.org/wiki/Network_Time_Protocol) server/client to synchronize the client system time to the Windows system time or vis-versa.
