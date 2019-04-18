#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3, Pose
from e190_bot.srv import *
import tf

def handle_point_tracking(req):
    # initialize stuff
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()

    pos = req.goal.position
    orientation = req.goal.orientation

    rate = rospy.Rate(200.0)

    # get params for proportional control
    kp = rospy.get_param('kp', 0.5)
    kb = rospy.get_param('kb', -1.0)
    ka = rospy.get_param('ka', 0.7)

    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()

            # first broadcast goal transform
            broadcaster.sendTransform((pos.x, pos.y, pos.z),
                (orientation.x, orientation.y, orientation.z, orientation.w),
                rospy.Time.now(), "/goal", "/odom")

            # Get transform from tf
            listener.waitForTransform("/base_link", "/goal", now, rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform("/base_link", "/goal", now)

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

            # Publish velocity to control.py
            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            print("sending command: %s" % (cmd))
            pub.publish(cmd)
            rate.sleep()
        except (tf.LookupException, tf.ConnectivityException):
            return point_trackingResponse(False)

def point_tracking_server():
    rospy.init_node('point_tracking_server')
    pts = rospy.Service('point_tracking', point_tracking, handle_point_tracking)
    print("Ready to track points.")
    rospy.spin()

if __name__ == '__main__':
    point_tracking_server()
