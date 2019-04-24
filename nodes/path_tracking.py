#!/usr/bin/env python

import sys
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Twist, Vector3
from e190_bot.srv import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf

class path_tracking():
    def __init__(self):
        rospy.init_node('path_tracking')
        rospy.Subscriber("/plan", Path, self.plan_callback)

        # self.rate = rospy.Rate(10)
        self.rate = rospy.Rate(500)
        print('successful init')

        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.rot_cmd = Twist()
        self.rot_cmd.angular.z = 5

        while not rospy.is_shutdown():
            self.rate.sleep();


    def plan_callback(self, plan):
        print("reached plan callback")
        # Make sure service is available
        rospy.wait_for_service('set_goal')
        try:
            setGoal = rospy.ServiceProxy('set_goal', set_goal)
            for pose in plan.poses:
                resp = False

                rot_next = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
                next_orientation = euler_from_quaternion(rot_next)[2]
                print("before while")
                while True:
                    try:
                        now = rospy.Time.now()
                        print("waiting for transform . . .")
                        self.listener.waitForTransform("/odom", "/base_link", now, rospy.Duration(4.0))
                        print("waited")
                        (trans, rot) = self.listener.lookupTransform("/odom","/base_link", now)

                        print("got transform")
                        curr_orientation = euler_from_quaternion(rot)[2]

                        print("orientation is: ",curr_orientation, next_orientation)

                        if (abs(next_orientation - curr_orientation) < .1):
                            break;
                        else:
                            self.pub.publish(self.rot_cmd)

                        self.rate.sleep();

                    except (tf.LookupException, tf.ConnectivityException):
                        pass

                self.pub.publish(Twist())

                while(not resp):
                    resp = setGoal(pose.pose)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


if __name__ == "__main__":
    print('in path tracking')
    path_tracking()
