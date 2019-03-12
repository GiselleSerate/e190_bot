#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import Transform, Twist, Quaternion, Vector3
from fiducial_msgs.msg import FiducialTransformArray
import tf


class steering():
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('steering', anonymous=True)

        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.steering_callback)

        rate = rospy.Rate(10)

        self.forward_constraint = 0.5
        self.backwards_constraint = 0.6
        self.linear_scalar = -1.0
        self.rotational_scalar = 1.0

        self.vel = Twist()
        self.vel.linear = Vector3(0,0,0)
        self.vel.angular = Vector3(0,0,0)

        while not rospy.is_shutdown():
            self.pub_cmd_vel()
            rate.sleep()


    def steering_callback(self, transforms):
        if len(transforms.transforms) > 0:
            raw_linear = transforms.transforms[0].transform.translation.z
            raw_rot = transforms.transforms[0].transform.rotation
            rot = np.array([raw_rot.x, raw_rot.y, raw_rot.z, raw_rot.w])
            raw_angular = tf.transformations.euler_from_quaternion(rot)[2]

            self.vel.linear.x = .5 + self.linear_scalar*(raw_linear-0.3)
            self.vel.angular.z = raw_angular * self.rotational_scalar

    def pub_cmd_vel(self):
        self.pub.publish(self.vel)

if __name__ == '__main__':
    try:
        steering()
    except rospy.ROSInterruptException:
        pass
