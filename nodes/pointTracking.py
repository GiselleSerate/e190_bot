#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3
import tf

# kp = 3.0
# ka = 0.0
# kb = -4.0

# kp = 1.5
# ka = 0.0
# kb = -2.0

# kp = 1
# ka = 1
# kb = -2.0

# # Strong stability control (assume kb < 0)
# kp = 0.5
# ka = 1
# kb = -0.2
# # Slow, but it got there. 

# # Inc kp, ka. 
# kp = 0.7
# ka = 1.2
# kb = -0.2
# # too fast forward

# # dec kp, inc ka. 
# kp = 0.5
# ka = 1.5
# kb = -0.2
# # doesn't seem to be stable

# # Strong stability control, assuming slides correct
# kp = 0.5
# ka = -0.5
# kb = -0.2
# # wanders off towards the right instead of going towards goal

# # Normal stability control as suggested in the slides
# kp = 3
# ka = 7
# kb = -1.5
# # straight line

# # Normal stability control
# kp = 0.5
# ka = 1
# kb = -1.5
# # 1m to the left but not fwd

# # Normal stability control
# kp = 0.5
# ka = 1
# kb = -3
# # further backwards than previous

# # Retrying
# kp = 1
# ka = 1
# kb = -2.0
# # escaping, bye

# # Retrying with smaller kp
# kp = 0.5
# ka = 1
# kb = -2.0
# # not far enough +x in 1,1,0; almost perf in 1,0,0

# # Retrying with smaller ka
# kp = 0.5
# ka = 0.7
# kb = -2.0
# # overshooting

# # Retrying with smaller kb
# kp = 0.5
# ka = 0.7
# kb = -1.0
# # Seem like good consts

# # Retrying with larger kp
# kp = 0.6
# ka = 0.7
# kb = -1.0
# # 

# # Retrying with bigger ka
# kp = 0.6
# ka = 0.9
# kb = -1.0
# # bad, go back

# Roll back to good
kp = 0.5
ka = 0.7
kb = -1.0

def pointTracking():
    print("in pointTracking\n");
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('pointTracking', anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(200.0)
    rospy.sleep(5)

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

            if rho < 0.02:
                linear = .0
                angular = .0


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
