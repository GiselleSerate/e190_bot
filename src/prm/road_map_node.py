from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

class PRM_Node:

    def __init__(self, x = 0, y = 0, parent = None, children = [], index = 0):
        self.x = x
        self.y = y
        self.parent = parent
        self.children = children
        self.index = index #frankly, pretty useless if we use strict linked graph
        #Feel free to add other member variables if you think you need!

    def addChild(self, childNode):
        self.children.append(childNode)
        #python doesn't have the concept of pointer, it shows drawback here!!!
        #Yes, we like C!
        childNode.parent = self

    def pose(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0

        quat = quaternion_from_euler(.0, .0, Math.atan2(self.y-self.parent.y, self.x-self.parent.x))
        pose.pose.orientation = Quaternion()
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
