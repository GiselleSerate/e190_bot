#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool
import tf

# P control constants
kp = rospy.get_param("kp", 0.5)
ka = rospy.get_param("ka", 0.7)
kb = rospy.get_param("kb", -1.0)



def pointTracking():
    print("in pointTracking")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    reachPub = rospy.Publisher('/goal_reached', Bool, queue_size=10)
    rospy.init_node('pointTracking', anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(200.0)
    rospy.sleep(5)

    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()

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
                reached = True
            else:
                reached = False

            reachPub.publish(reached)

            # Publish velocity to control.py
            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            pub.publish(cmd)
            rate.sleep()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

if __name__ == '__main__':
    try:
        pointTracking()
    except rospy.ROSInterruptException:
        pass
