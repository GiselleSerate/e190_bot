#!/usr/bin/env python
import roslib

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

import rospy
import tf


class defineGoal:
    def __init__(self):
        rospy.init_node('point_tracking_server')
        self.rate = rospy.Rate(10.0)

        self.goal = Pose((2,0,0), tf.transformations.quaternion_from_euler(.0, .0, 0))
        self.reached = False
        
        rospy.Subscriber("/goal_reached", Bool, self.handle_reached)
        self.broadcaster = tf.TransformBroadcaster()
        
        self.s = rospy.Service('point_tracking', point_tracking, self.handle_point_tracking)
        
        while not rospy.is_shutdown():
            # Send tf transform for pointTracking to handle
            self.broadcaster.sendTransform(self.goal.position, self.goal.orientation, rospy.Time.now(), "/goal", "/odom")
            self.rate.sleep()

    def handle_reached(self, reached):
        self.reached = reached

    def handle_point_tracking(self, req):
        print "Aiming for goal:"
        print req.dest
        # Sleep until goal reached
        while not self.reached:
            self.rate.sleep()
        return self.reached

if __name__ == "__main__":
    defineGoal()