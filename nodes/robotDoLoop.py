#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3

forward_velocity = .5        # m/sec
rotate_velocity = math.pi # rad/sec

def robotDoLoop():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('robotDoLoop', anonymous=True)

    do_nothing = Twist();
    do_nothing.linear = Vector3(0, 0, 0)
    do_nothing.angular = Vector3(0, 0, 0)

    go_forward = Twist();
    go_forward.linear = Vector3(forward_velocity, 0, 0)
    go_forward.angular = Vector3(0, 0, 0)

    do_rotation = Twist()
    do_rotation.linear = Vector3(0, 0, 0)
    do_rotation.angular = Vector3(0, 0, rotate_velocity)

    cmds = [do_nothing, go_forward, do_nothing, do_rotation]*4 + [do_nothing]
    cmd_step = 0
    sleep_times = {do_nothing: 0.5, go_forward: 2.0, do_rotation: 0.5}

    # Wait 5 seconds, then run the loop
    rospy.sleep(5)

    while not rospy.is_shutdown() and cmd_step < len(cmds):
        cmd = cmds[cmd_step]
        sleep_time = sleep_times[cmd]
        # rospy.loginfo(cmd, sleep_time)
        pub.publish(cmd)
        rospy.sleep(sleep_time)
        cmd_step += 1

if __name__ == '__main__':
    try:
        robotDoLoop()
    except rospy.ROSInterruptException:
        pass
