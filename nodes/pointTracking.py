#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3
import tf

# kp = 3.0
# ka = 0.0
# kb = -4.0

kp = 1.5
ka = 0.0
kb = -2.0


def pointTracking():
    print("in pointTracking\n");
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('pointTracking', anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(200.0)
    rospy.sleep(5) # TODO: maybe don't need

    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform("/base_link", "/goal", now, rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform("/base_link", "/goal", now)

            theta = tf.transformations.euler_from_quaternion(rot)[2]

            rho = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            beta = -1 * math.atan2(trans[1], trans[0])
            alpha = -1 * beta - theta

            angular = ka * alpha + kb * beta
            linear = kp * rho

            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            pub.publish(cmd)
            rate.sleep()
        except (tf.LookupException, tf.ConnectivityException): 
            pass

if __name__ == '__main__':
    try:
        pointTracking()
    except rospy.ROSInterruptException:
        pass