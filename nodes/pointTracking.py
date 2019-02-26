#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3
import tf

# kp = 
# ka = 
# kb = 


def pointTracking():
    print("in pointTracking\n");
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('pointTracking', anonymous=True)
    listener = tf.TransformListener()
    # print("inits and stuff finished\n");


    # print("broadcaster did a broadcast\n");

    # listener.waitForTransform("/base_link", "/odom", rospy.Time(), rospy.Duration(4.0))
    
    # print("waited on baselink to odom\n");

    # listener.waitForTransform("/goal", "/odom", rospy.Time(), rospy.Duration(4.0))
    
    # print("waited on goal to odom\n");

    # listener.waitForTransform("/base_link", "/goal", rospy.Time(), rospy.Duration(4.0))
    # print("waited on baselink to goal\n");
    rate = rospy.Rate(100.0)
    rospy.sleep(5)

    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform("/base_link", "/goal", now, rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform("/base_link", "/goal", now)

            angular = 4 * math.atan2(trans[1], trans[0])
            linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)

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