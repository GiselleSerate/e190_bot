#!/usr/bin/env python
import roslib

import rospy
from e190_bot.srv import *
from geometry_msgs.msg import Twist, Vector3, Pose
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler
import tf

class set_goal_server():
    def __init__(self):
        rospy.init_node('set_goal_server')
        self.broadcaster = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10.0)

        self.position = (0,0,0)
        self.orientation = quaternion_from_euler(0, 0, 0)
        self.reached = True

        pts = rospy.Service('set_goal', set_goal, self.handle_set_goal)

        rospy.Subscriber("/reached_goal", Bool, self.handle_goal_reached)


        while not rospy.is_shutdown():
            broadcaster.sendTransform(self.position, self.orientation, rospy.Time.now(), "/goal", "/odom")
            self.rate.sleep()


    def handle_set_goal(self, req):
        self.reached = False
        pos = req.goal.position
        orientation = req.goal.orientation
        self.position = (pos.x, pos.y, pos.z)
        self.orientation = (orientation.x, orientation.y, orientation.z, orientation.w)

        while not self.reached:
            self.rate.sleep()

        return set_goalResponse(True)

    def handle_goal_reached(self, reached):
        self.reached = reached.data


if __name__ == '__main__':
    set_goal_server()
