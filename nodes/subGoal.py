#!/usr/bin/env python
import roslib

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
    	# Send hard-coded transform
        broadcaster.sendTransform((2,0,0), tf.transformations.quaternion_from_euler(.0, .0, 0), rospy.Time.now(), "/goal", "/odom")
        rate.sleep()
