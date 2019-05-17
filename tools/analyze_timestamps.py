#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

from math import sqrt


class TimestampAnalyzer:
    def __init__(self):
        window = rospy.get_param("window", default=10)

        self.mean_dt = 0.0
        self.variance = 0.0
        self.count = 0

        self.last_t = None

        self.sub = rospy.Subscriber(
            "topic", PoseStamped, self.callback, queue_size=5)

        self.timer = rospy.Timer(rospy.Duration(
            window), self.timer_callback, oneshot=True)

        print("Listening to topic '{}' for {} seconds...".format(
            self.sub.resolved_name, window))

    def callback(self, msg):
        if self.last_t is None:
            self.last_t = msg.header.stamp
            return

        dt = (msg.header.stamp - self.last_t).to_sec()
        self.last_t = msg.header.stamp

        self.count += 1
        self.mean_dt = ((self.count - 1) * self.mean_dt + dt) / self.count

        self.variance = (self.count - 1)/self.count * \
            self.variance + 1.0/self.count * (dt - self.mean_dt)**2

    def timer_callback(self, event):
        self.sub.unregister()

        std_deviation = sqrt(self.variance)

        print('Received {} messages.'.format(self.count))

        if self.count > 0:
            print('Mean dt: {} s ({} Hz)'.format(
                self.mean_dt, 1.0/self.mean_dt))
            print('Standard Deviation: {} s ({:.1f}% of mean)'.format(
                std_deviation, std_deviation / self.mean_dt * 100))

        rospy.signal_shutdown("Done")


if __name__ == '__main__':
    rospy.init_node("analyze_timestamps")
    analyzer = TimestampAnalyzer()
    rospy.spin()
