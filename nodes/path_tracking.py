#!/usr/bin/env python

import sys
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from e190_bot.srv import *

class path_tracking():
    def __init__(self):
        rospy.init_node('path_tracking')
        rospy.Subscriber("/plan", Path, self.plan_callback)
        self.rate = rospy.Rate(10)
        print('successful init')

        while not rospy.is_shutdown():
            self.rate.sleep();


    def plan_callback(self, plan):
        print("reached plan callback")
        # Make sure service is available
        rospy.wait_for_service('point_tracking')
        try:
            pointTracking = rospy.ServiceProxy('point_tracking', point_tracking)

            for pose in plan.poses:
                resp = False
                while(not resp):
                    resp = pointTracking(pose.pose)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


if __name__ == "__main__":
    print('in path tracking')
    path_tracking()
