#!/usr/bin/env python
import rospy
import rospkg
import random

import tf

from road_map_node import PRM_Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

import math

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class prm_planning:

	def __init__(self):

		rospy.init_node('prm_planning', anonymous=True)

		# subscribe to /goal topic, you can use "2D Nav Goal" tab on RViz to set a goal by mouse
		# config 2D Nav Goal using panels->tool properties
		rospy.Subscriber("/goal", PoseStamped, self.goal_callback)
		rospy.Subscriber("/odom", Odometry, self.odom_callback)

		self.pubPlan = rospy.Publisher('/plan', Path, queue_size=10)

		self.path_init()
		self.map_init()# call map_server using service, other methods possible

		self.start_i = 0
		self.start_j = 0
		self.start_id = 0 #not important most of time


		self.goal_i = 0
		self.goal_j = 0
		self.goal_id = 0 #not important most of time


		self.current_x = .0
		self.current_y = .0
		quat = quaternion_from_euler(.0, .0, .0)
		self.current_o = Quaternion()
		self.current_o.x = quat[0]
		self.current_o.y = quat[1]
		self.current_o.z = quat[2]
		self.current_o.w = quat[3]

		self.rate = rospy.Rate(2)
		while not rospy.is_shutdown():
			self.rate.sleep();


	def path_init(self):
		self.prm_plan = Path()
		self.prm_plan.header.frame_id = "map"
		self.roadmap = []
		self.start_node = PRM_Node()
		self.goal_node = PRM_Node()
		self.randomRes = 0.5

	def map_init(self):
		rospy.wait_for_service('static_map')
		try:
			map_Service = rospy.ServiceProxy('static_map', GetMap)
			self.map = map_Service().map
			self.map_width = self.map.info.width
			self.map_height = self.map.info.height
			self.map_res = self.map.info.resolution

			print("Map size: " + str(self.map_res*self.map_width) +" , " + str(self.map_res*self.map_height))
		except rospy.ServiceException, e:
			print "Map service call failed: %s"%e

	def goal_callback(self,Goal):
		self.goal_x = Goal.pose.position.x
		self.goal_y = Goal.pose.position.y
		self.goal_o = Goal.pose.orientation

		self.start_x = self.current_x
		self.start_y = self.current_y
		self.start_o = self.current_o

		print("goal_clicked: " + str(self.goal_x) + " , " + str(self.goal_y))

		self.goal_i, self.goal_j, self.goal_id = self.pos_to_grid(self.goal_x, self.goal_y)
		self.start_i, self.start_j, self.start_id = self.pos_to_grid(self.start_x,self.start_y)


		self.plan_path()

		self.pubPlan.publish(self.prm_plan) #plan is published here!

	def odom_callback(self,Odom):
		# When you are using your actual robot, you need to update this
		# Let's assume it starts at 0

		self.current_x = .0
		self.current_y = .0

		quat = quaternion_from_euler(.0, .0, .0)
		self.current_o = Quaternion()
		self.current_o.x = quat[0]
		self.current_o.y = quat[1]
		self.current_o.z = quat[2]
		self.current_o.w = quat[3]

	def create_new_node(self, base_node):
		collisionFree = False

		# print("[")
		while(not collisionFree):
			randX = random.uniform(-self.randomRes, self.randomRes)
			randY = random.uniform(-self.randomRes, self.randomRes)

			new_node = PRM_Node(x=base_node.x+randX,y=base_node.y+randY,parent=base_node)
			collisionFree = self.collisionDetect(base_node.x, base_node.y, new_node.x, base_node.y)
		# print("]")

		return new_node

	def plan_path(self):
		# Core function! modify as you wish! Here is only a demo that yield definitely wrong thing
		# Here is an example how do you deal with ROS nav_msgs/Path

		nodes = []

		start_pose = PoseStamped()
		start_pose.header.frame_id = "map"
		start_pose.pose.position.x = self.start_x
		start_pose.pose.position.y = self.start_y
		start_pose.pose.position.z = 0

		start_pose.pose.orientation = self.start_o

		goal_pose = PoseStamped()
		goal_pose.header.frame_id = "map"
		goal_pose.pose.position.x = self.goal_x
		goal_pose.pose.position.y = self.goal_y
		goal_pose.pose.position.z = 0

		goal_pose.pose.orientation = self.goal_o

		nodes.append(PRM_Node(x=start_pose.pose.position.x, y=start_pose.pose.position.y))

		print("Starting to create nodes")
		pathToGoal = self.collisionDetect(self.start_x, self.start_y, self.goal_x, self.goal_y)
		while(not pathToGoal):
			new_node = self.create_new_node(nodes[random.randint(0, len(nodes)-1)])
			new_node.index = len(nodes)
			nodes[-1].addChild(new_node)
			nodes.append(new_node)
			pathToGoal = self.collisionDetect(new_node.x, new_node.y, self.goal_x, self.goal_y)
		print("Found path to goal!")

		print("Optimizing")
		# optimization
		short_cut_index = 0
		last_node_index = len(nodes) - 1
		while (short_cut_index != last_node_index):
			curr_node_index = last_node_index
			short_cut_node = nodes[short_cut_index]
			while True:
				curr_node = nodes[curr_node_index]
				if (curr_node.parent.index == short_cut_index):
					short_cut_index = curr_node.index
					break

				# check collision
				is_short_cut = self.collisionDetect(short_cut_node.x, short_cut_node.y, curr_node.x, curr_node.y)
				if (is_short_cut):
					print("Optimization found!")
					short_cut_node.addChild(curr_node)
					short_cut_index = curr_node.index
					break

				# update indices
				curr_node_index = curr_node.parent.index
		print("Done optimizing")

		# search
		self.prm_plan.poses.insert(0, goal_pose)
		curr_node = nodes[-1]
		print(len(nodes))
		print(curr_node.parent)
		while(curr_node.parent != None):
			self.prm_plan.poses.insert(0, curr_node.pose())
			curr_node = curr_node.parent
		self.prm_plan.poses.insert(0, start_pose)

		print("I have: " + str(len(self.prm_plan.poses)) + " poses in path planned")

	#convert position in meter to map grid id, return grid_x, grid_y and their 1d grid_id
	def pos_to_grid(self,poseX,poseY):
		grid_i = int(round((poseX - self.map.info.origin.position.x) / self.map_res));
		grid_j = int(round((poseY - self.map.info.origin.position.y) / self.map_res));

		grid_id = grid_j * self.map_width + grid_i

		return grid_i, grid_j, grid_id


	#straight line collision detection, all inputs unit are in meter
	# Returns True if collision free.
	def collisionDetect(self,x1,y1,x2,y2):
		grid_i1, grid_j1, grid_id1 = self.pos_to_grid(x1, y1)
		grid_i2, grid_j2, grid_id2 = self.pos_to_grid(x2, y2)

		# print ("Check line between: (%d, %d) and (%d, %d)" % (grid_i1, grid_j1, grid_i2, grid_j2))
		line = bresenham(grid_i1, grid_j1, grid_i2, grid_j2)
		for k in range(0,len(line)):
			# print("Check map gird: " + str(line[k][0]) + " " + str(line[k][1]))
			# print(self.map.data[line[k][1] * self.map_width + line[k][0]])

			#Map value 0 - 100
			if(self.map.data[line[k][1] * self.map_width + line[k][0]]>85):
				return False

		return True

# bresenham algorithm for line generation, from http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
def bresenham(x1, y1, x2, y2):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end

    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions

    dx = x2 - x1
    dy = y2 - y1

    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)

    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1

    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()

    # print points
    return points


if __name__ == '__main__':
	try:
		planning = prm_planning()

	except rospy.ROSInterruptException:
		pass
