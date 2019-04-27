#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3, Pose
from e190_bot.srv import *
from tf.transformations import quaternion_from_euler
import tf

class point_tracking_server():
    def __init__(self):
        rospy.init_node('point_tracking_server')

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(200.0)

        self.position = (0,0,0)
        self.orientation = quaternion_from_euler(0, 0, 0)
        self.stopped = False

        pts = rospy.Service('point_tracking', point_tracking, self.handle_point_tracking)

        while not rospy.is_shutdown():
            # first broadcast goal transform
            self.broadcaster.sendTransform(self.position,
                self.orientation,
                rospy.Time.now(), "/goal", "/odom")
            self.rate.sleep();

    def handle_point_tracking(self, req):
        pos = req.goal.position
        orientation = req.goal.orientation

        self.position = (pos.x, pos.y, pos.z)
        self.orientation = (orientation.x, orientation.y, orientation.z, orientation.w)

        # get params for proportional control
        kp = rospy.get_param('kp', 0.5)
        kb = rospy.get_param('kb', -1.0)
        ka = rospy.get_param('ka', 0.7)

        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()

                # Get transform from tf
                self.listener.waitForTransform("/base_link", "/goal", now, rospy.Duration(4.0))
                (trans,rot) = self.listener.lookupTransform("/base_link", "/goal", now)

                # Get P control distances and angles
                theta = tf.transformations.euler_from_quaternion(rot)[2]
                rho = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
                beta = -1 * math.atan2(trans[1], trans[0])
                alpha = -1 * beta - theta

                # Calculate velocities
                angular = ka * alpha + kb * beta
                linear = kp * rho

                # Stop driving when close enough to goal
                if rho < 0.02:
                    linear = .0
                    angular = .0
                    return point_trackingResponse(True)

                # Already stopped. no need to stop further.
                if (self.stopped and linear == 0 and angular == 0):
                    print("already stopped")
                    continue

                # Publish velocity to control.py
                cmd = Twist()
                cmd.linear.x = linear
                cmd.angular.z = angular
                print("sending command: %s" % (cmd))
                self.pub.publish(cmd)
                self.stopped = (linear == 0 and angular == 0)
                print("stopped is:", self.stopped)
                self.rate.sleep()
            except (tf.LookupException, tf.ConnectivityException):
                return point_trackingResponse(False)


if __name__ == '__main__':
    point_tracking_server()
