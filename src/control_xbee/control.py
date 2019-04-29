#!/usr/bin/env python
import rospy
import rospkg
from xbee import XBee
import serial
import tf
import math
import numpy as np

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from e190_bot.msg import ir_sensor
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# from e190_bot.src.particle_filter.E160_PF import *



import math
import random
import numpy as np
import copy
# from e190_bot.src.particle_filter.E160_state import*



class E160_state:

    def __init__(self):
        self.set_state(0,0,0)

    def set_state(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta


from scipy.stats import norm
# from e190_bot.src.prm.prm_planning import bresenham
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


from numpy import mean
from nav_msgs.srv import GetMap

class E160_PF:

    def __init__(self, botRadius, wheel_radius, encoder_resolution):
        self.particles = []
        self.numParticles = 400

        # maybe should just pass in a robot class?
        self.bot_radius = botRadius
        self.wheel_radius = wheel_radius
        self.encoder_resolution = encoder_resolution
        self.FAR_READING = 1000

        # PF parameters
        self.IR_sigma = 0.2 # Range finder s.d
        self.odom_xy_sigma = 1.25   # odometry delta_s s.d
        self.odom_heading_sigma = 0.75  # odometry heading s.d
        self.odom_wheel_sigma = 0.01
        self.particle_weight_sum = 0
        self.variance = 0.02 # TODO tune this variance larger? idk lol
                             # maybe related to grid map resolution?

        # define the sensor orientations
        self.sensor_orientations = [0, math.pi/2, -math.pi/2]

        # initialize the current state
        self.state = E160_state()
        self.state.set_state(0,0,0)

        # Get the map
        rospy.wait_for_service('static_map')
        try:
            map_Service = rospy.ServiceProxy('static_map', GetMap)
            self.map = map_Service().map
            self.map_width = self.map.info.width
            self.map_height = self.map.info.height
            self.map_res = self.map.info.resolution
        except rospy.ServiceException, e:
            print "Map service call failed: %s"%e

        # TODO: change this later
        self.map_minX = self.map.info.origin.position.x
        self.map_maxX = self.map_minX + (self.map_width * self.map_res)
        self.map_minY = self.map.info.origin.position.y
        self.map_maxY = self.map_minX + (self.map_height * self.map_res)
        self.InitializeParticles()
        self.last_encoder_measurements =[0,0]

        self.odom_broadcaster = tf.TransformBroadcaster()


    def InitializeParticles(self):
        ''' Populate self.particles with random Particle
            Args:
                None
            Return:
                None'''
        self.particles = []
        for i in range(0, self.numParticles):
            #self.SetRandomStartPos(i)
            self.SetKnownStartPos()


    def SetRandomStartPos(self, i):
        i.x = random.random() * (self.map_maxX - self.map_minX) + self.map_minX
        i.y = random.random() * (self.map_maxY - self.map_minY) + self.map_minY

    def SetKnownStartPos(self):
        particle = E160_PF.Particle(self.state.x, self.state.y, self.state.theta, 0)
        self.particles.append(particle)

    def LocalizeEstWithParticleFilter(self, diffEncoderL, diffEncoderR, sensor_readings):
        ''' Localize the robot with particle filters. Call everything
            Args:
                delta_s (float): change in distance as calculated by odometry
                delta_heading (float): change in heading as calcualted by odometry
                sensor_readings([float, float, float]): sensor readings from range fingers
            Return:
                estimated position'''

        newDiffL = random.gauss(diffEncoderL, self.odom_wheel_sigma)
        newDiffR = random.gauss(diffEncoderR, self.odom_wheel_sigma)

        # Calculate changes in theta and distance with respect to the robot frame
        delta_heading = ((newDiffR - newDiffL) * self.wheel_radius)/(2.0 * self.bot_radius)
        delta_s = ((newDiffR + newDiffL) * self.wheel_radius)/2.0

        for particle in self.particles:
            self.Propagate(delta_s, delta_heading, particle)
            self.CalculateWeight(sensor_readings, particle)


        return self.GetEstimatedPos()

    def Propagate(self, delta_s, delta_heading, i):
        '''Propagate all the particles from the last state with odometry readings
            Args:
                delta_s (float): distance traveled based on odometry
                delta_heading(float): change in heading based on odometry
            return:
                nothing'''
        # previous signature: def Propagate(self, encoder_measurements, i):

        i.x += delta_s * math.cos(i.heading + delta_heading / 2.0)
        i.y += delta_s * math.sin(i.heading + delta_heading / 2.0)

        i.heading += delta_heading
        i.heading = i.heading % (2 * math.pi)


    def CalculateWeight(self, sensor_readings, particle):
        '''Calculate the weight of a particular particle
            Args:
                particle (E160_Particle): a given particle
                sensor_readings ( [float, ...] ): readings from the IR sesnors
                walls ([ [four doubles], ...] ): positions of the walls from environment,
                            represented as 4 doubles
            return:
                new weight of the particle (float) '''

        weights = []
        for i in range(len(self.sensor_orientations)):
            wall_dist = self.FindMinWallDistance(particle, self.sensor_orientations[i])
            if wall_dist > 0 and sensor_readings[i] > 0:
                weights.append(norm.pdf(sensor_readings[i], wall_dist, self.variance))
            # elif wall_dist > 0 and sensor_readings[i] < 0:
            #     # This is okay. maybe the sensor is bugged out for a second.
            #     weights.append(norm.pdf(wall_dist + self.variance, wall_dist, self.variance))
            # else:
            #     # less good
            #     weights.append(norm.pdf(wall_dist + 2 * self.variance, wall_dist, self.variance))

        if len(weights) != 0:
            newWeight = mean(weights)
        else:
            newWeight = norm.pdf(self.variance/2, 0, self.variance)
        particle.weight = newWeight
        return newWeight

    def Resample(self):
        '''Resample the particles systematically
            Args:
                None
            Return:
                None'''

        # TODO not sure if dividing by numParticles is the right move here, what are weights??
        self.particles = np.random.choice(self.particles, size=self.numParticles, replace=True, p=[particle.weight / self.numParticles for particle in self.particles])


    def GetEstimatedPos(self):
        ''' Calculate the mean of the particles and return it
            Args:
                None
            Return:
                estimated position'''

        # TODO am skeptical about literally averaging all the coordinates but Okay
        print("particles number:", len(self.particles))
        self.state.x = mean([particle.x for particle in self.particles])
        self.state.y = mean([particle.y for particle in self.particles])
        self.state.heading = mean([particle.heading for particle in self.particles])

        self.odom_broadcaster.sendTransform(
            (self.state.x, self.state.y, .0),
            tf.transformations.quaternion_from_euler(.0, .0, self.state.heading),
            rospy.Time.now(),
            "/base_link_est",
            "/odom",
        )

        return self.state


    def FindMinWallDistance(self, particle, sensorT):
        ''' Given a particle position, walls, and a sensor, find
            shortest distance to the wall
            Args:
                particle (E160_Particle): a particle
                sensorT: orientation of the sensor on the robot (assuming radians)
            Return:
                distance to the closest wall' (float)'''
        sensorO = (particle.heading + sensorT) % (2 * math.pi)
        # This is a modified version of the collision function

        if (particle.x >= self.map_maxX or particle.x < self.map_minX or particle.y >= self.map_maxY or particle.y < self.map_minY):
            return -1

        slope = math.tan(sensorO)
        b = particle.y - (slope * particle.x)

        x2 = 0
        y2 = 0
        # pick out which side we might hit
        sideX = self.map_minX if sensorO > math.pi/2 else self.map_maxX
        sideY = b + (slope * sideX)

        # protect against zero divison
        if not slope:
            x2 = sideX
            y2 = sideY
        else:
            # pick out which vertical limit we might hit
            verticalY = self.map_minY if sensorO > math.pi else self.map_maxY
            verticalX = (verticalY - b)/slope

            # if our slope is 0 then we just pick the side value, if 1, then the vert
            if slope == 1:
                x2 = verticalX
                y2 = verticalY
            # if the X value of the vertical or the Y value of the side value
            # are not within range, then set x2 and y2 to be the others
            elif sideY > self.map_maxX or sideY < self.map_minX:
                x2 = verticalX
                y2 = verticalY
            else:
                x2 = sideX
                y2 = sideY

        grid_i1, grid_j1, grid_id1 = self.pos_to_grid(particle.x, particle.y)
        grid_i2, grid_j2, grid_id2 = self.pos_to_grid(x2, y2)

        intersectionD = -1 # -1 if no intersection
        line = bresenham(grid_i1, grid_j1, grid_i2, grid_j2)
        for k in range(0, len(line)):
            # Map value 0 - 100
            if(self.map.data[line[k][1] * self.map_width + line[k][0]] > 85):
                # get the first collision
                # (gets distance to middle of gridpoint)
                xd = abs(particle.x - ((line[k][0] + .5) * self.map_res) + self.map_minX)
                yd = abs(particle.y - ((line[k][1] + .5) * self.map_res) + self.map_minY)
                intersectionD = math.sqrt(math.pow(xd, 2) + math.pow(yd, 2))
                break

        return intersectionD

        # convert position in meter to map grid id, return grid_x, grid_y and their 1d grid_id
    def pos_to_grid(self, poseX, poseY):
        grid_i = int((poseX - self.map.info.origin.position.x) / self.map_res)
        grid_j = int((poseY - self.map.info.origin.position.y) / self.map_res)
        grid_id = grid_j * self.map_width + grid_i
        return grid_i, grid_j, grid_id


    def angleDiff(self, ang):
        ''' Wrap angles between -pi and pi'''
        while ang < -math.pi:
            ang = ang + 2 * math.pi
        while ang > math.pi:
            ang = ang - 2 * math.pi
        return ang

    class Particle:
        def __init__(self, x, y, heading, weight):
            self.x = x
            self.y = y
            self.heading = heading
            self.weight = weight

        def __str__(self):
            return str(self.x) + " " + str(self.y) + " " + str(self.heading) + " " + str(self.weight)




rospack = rospkg.RosPack()

class botControl:
    """A class for controlling a two wheeled robot with three IR sensors and three encoders.
    This class communicates with the robot using an xbee enabling the robot to move, rotate
    and view the environemnt with IR sensors."""

    def __init__(self):
        "Initializes communication over the xbee and inital hardware values."
        # create vars for hardware vs simulation
        self.robot_mode = "HARDWARE_MODE"#"SIMULATION_MODE"
        #self.control_mode = "MANUAL_CONTROL_MODE"

        # Measured bot parameters
        self.wheel_radius = 0.035
        self.bot_radius = 0.07

        # hardcoded command velocities
        self.stopBot = Twist()
        self.stopBot.linear = Vector3(0, 0, 0)
        self.stopBot.angular = Vector3(0, 0, 0)


        # setup xbee communication, change ttyUSB0 to the USB port dongle is in
        if (self.robot_mode == "HARDWARE_MODE"):
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)
            print(" Setting up serial port")
            try:
                self.xbee = XBee(self.serial_port)
            except:
                print("Couldn't find the serial port")

        print("Xbee setup successful")
        self.address = '\x00\x0C'#you may use this to communicate with multiple bots

        # Configure IR sensors
        self.ir_init()

        #init an odometry instance, and configure odometry info
        self.odom_init()

        #init log file, "False" indicate no log will be made, log will be in e190_bot/data folder
        self.log_init(data_logging=True,file_name="log.txt")

        # Creates ROS nodes and a topic to control movement
        rospy.init_node('botControl', anonymous=True)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        self.pubOdom = rospy.Publisher('/odom', Odometry, queue_size=10)

        self.pubDistL = rospy.Publisher('/distL', ir_sensor, queue_size=10)
        self.pubDistC = rospy.Publisher('/distC', ir_sensor, queue_size=10)
        self.pubDistR = rospy.Publisher('/distR', ir_sensor, queue_size=10)
        self.time = rospy.Time.now()
        self.count = 0;

        # Sets publishing rate
        self.rate = rospy.Rate(10) # 10hz
        self.xbeeTimeout = .01 # set an xbee timeout such that we only skip one
                              # or two odom pubs
        # self.rate = rospy.Rate(5) # 5hz

        odomWorks = True
        while (not rospy.is_shutdown()):
            odomWorks = self.odom_pub();
            self.rate.sleep();

    def ir_init(self):
        """Initializes ir sensor messages for the three sensors on our bot."""
        self.ir_L = ir_sensor()
        self.ir_C = ir_sensor()
        self.ir_R = ir_sensor()

    def odom_init(self):
        """Initializes the odometer variables for distance measurements."""
        self.Odom = Odometry()
        self.Odom.header.frame_id = "/odom"
        self.Odom.child_frame_id = "/base_link"
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.encoder_resolution = 1.0/1440.0
        self.diffEncoderL = 0
        self.diffEncoderR = 0
        self.last_encoder_measurementL = 0
        self.last_encoder_measurementR = 0
        self.bot_angle = 0

        # TODO
        self.PF = E160_PF(self.bot_radius, self.wheel_radius, self.encoder_resolution)

        self.isFirstTime = True

    def log_init(self,data_logging=False,file_name="log.txt"):
        """Initializes logging of key events."""
        self.data_logging=data_logging
        if(data_logging):
            self.file_name = file_name
            self.make_headers();

    def calibrate(self,LAvel,RAvel):
        """Takes in left and right angular velocities of wheels and outputs
        left and right PWM values."""
        # Coefficients as calibrated
        LPWM = 8.52*abs(LAvel)+6.36
        RPWM = 8.61*abs(RAvel)+6.08

        scalar = min(255.0/max(LPWM, RPWM), 1)

        LPWM = int(scalar * LPWM)
        RPWM = int(scalar * RPWM)

        floor = 10

        if(LPWM < floor):
            LPWM = 0 # Floor left
        if(RPWM < floor):
            RPWM = 0 # Floor right

        return LPWM, RPWM

    def cmd_vel_callback(self,CmdVel):
        """Converts input: CmdVel a ROS message composed of two 3-vectors named
                linear and angular
            to values that the robot understands and sends them to the robot
            over the xbee.
            """
        if(self.robot_mode == "HARDWARE_MODE"):
            # Turn bot velocities into wheel velocities; keep as floats for now
            LAvel = (CmdVel.linear.x + CmdVel.angular.z * self.bot_radius) / self.wheel_radius
            RAvel = (CmdVel.linear.x - CmdVel.angular.z * self.bot_radius) / self.wheel_radius

            # Turn desired wheel velocities into PWMs
            LPWM, RPWM = self.calibrate(LAvel, RAvel)

            # Invert direction so +x is forward
            LDIR = int(LAvel < 0)
            RDIR = int(RAvel < 0)

            # Assemble command and send to terminal and robot
            command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            print(command)
            self.log_pwm(LPWM, RPWM)
            self.xbee.tx(dest_addr = self.address, data = command)

    def odom_pub(self):
        """Handles publishing of robot sensor data: sensor measurements and
        encoder measurements."""
        if(self.robot_mode == "HARDWARE_MODE"):
            self.count = self.count + 1
            print(self.count)

            command = '$S @'
            self.xbee.tx(dest_addr = self.address, data = command)
            try:
                update = self.xbee.wait_read_frame(self.xbeeTimeout)
            except:
                print("Lost connection to odom over xbee.\nRetrying")
                #self.cmd_vel_callback(self.stopBot)
                return False

            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            # Encoder readings as radians:
            encoder_measurementsL = data[4] * np.math.pi / 720.0 # TODO: Until we fix the Teensy code, compensate for switched encoders
            encoder_measurementsR = data[3] * np.math.pi / 720.0

            # Calculate how many radians the encoders have moved since the last odom measurement
            self.diffEncoderL = encoder_measurementsL - self.last_encoder_measurementL
            self.diffEncoderR = encoder_measurementsR - self.last_encoder_measurementR

            # Save last encoder measurements
            self.last_encoder_measurementL = encoder_measurementsL
            self.last_encoder_measurementR = encoder_measurementsR

            # oldMeasurement = 5

            # If either measurement is old, reset encoder differences
            if(self.isFirstTime):
                self.diffEncoderL = 0
                self.diffEncoderR = 0
                self.isFirstTime = False

            # Calculate changes in theta and distance with respect to the robot frame
            del_theta = ((self.diffEncoderR - self.diffEncoderL) * self.wheel_radius)/(2.0 * self.bot_radius)
            del_s = ((self.diffEncoderR + self.diffEncoderL) * self.wheel_radius)/2.0


            # Update x and y with deltas
            self.Odom.pose.pose.position.x += del_s * math.cos(self.bot_angle+del_theta/2.0)
            self.Odom.pose.pose.position.y += del_s * math.sin(self.bot_angle+del_theta/2.0)
            self.Odom.pose.pose.position.z = .0
            quat = quaternion_from_euler(.0, .0, self.bot_angle)
            self.Odom.pose.pose.orientation.x = quat[0]
            self.Odom.pose.pose.orientation.y = quat[1]
            self.Odom.pose.pose.orientation.z = quat[2]
            self.Odom.pose.pose.orientation.w = quat[3]

            self.bot_angle += del_theta
            self.bot_angle = self.bot_angle % (2*math.pi) # Loop

            # https://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
            self.odom_broadcaster.sendTransform(
                (self.Odom.pose.pose.position.x, self.Odom.pose.pose.position.y, .0),
                tf.transformations.quaternion_from_euler(.0, .0, self.bot_angle),
                rospy.Time.now(),
                self.Odom.child_frame_id,
                self.Odom.header.frame_id,
            )

            self.pubOdom.publish(self.Odom) # Publish in /odom topic

            range_measurements = data[:-2] # Range readings (there are 3)
            range_measurements = [self.ir_cal(val) for val in range_measurements]
            self.pubRangeSensor(range_measurements)

            # TODO
            self.state_est = self.PF.LocalizeEstWithParticleFilter(self.diffEncoderL, self.diffEncoderR, range_measurements)

        if(self.data_logging):
            self.log_data();

        self.time = rospy.Time.now()

        return True

    def ir_cal(self, val):
        """Convert ir sensor data to meters. The effective range of our sensors
        is from .2m to 1.5m. The best fit at this range was a power curve."""
        if val <= 0: # Prevent division by 0
            return 0.0
        measured_dist = 242.76 / (val ** 1.075)
        if measured_dist < .2 or measured_dist > 1.5:
            return -1.0
        else:
            return measured_dist


    def pubRangeSensor(self, ranges):
        """Publish the sensor recordings"""
        self.ir_L.distance = ranges[1]
        self.ir_C.distance = ranges[0]
        self.ir_R.distance = ranges[2]

        self.pubDistL.publish(self.ir_L)
        self.pubDistC.publish(self.ir_C)
        self.pubDistR.publish(self.ir_R)

    def make_headers(self):
        """Makes necessary headers for log file."""
        f = open(rospack.get_path('e190_bot')+"/data/"+self.file_name, 'a+')
        f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} {5:^1} \n'.format('TIME','X','Y','LIR','CIR','RIR'))
        f.close()

    def log_pwm(self, LPWM, RPWM):
        """Logs PWM changes for manual calibration."""
        f = open(rospack.get_path('e190_bot')+"/data/"+self.file_name, 'a+')

        data = [str(x) for x in ["------------------PWM-IS-NOW",LPWM,RPWM]]

        f.write(' '.join(data) + '\n')
        f.close()

    def log_data(self):
        """Logs data for debugging reference."""
        f = open(rospack.get_path('e190_bot')+"/data/"+self.file_name, 'a+')

        data = [str(x) for x in [self.time,self.Odom.pose.pose.position.x,self.Odom.pose.pose.position.y,self.ir_L.distance,self.ir_C.distance,self.ir_R.distance]]

        f.write(' '.join(data) + '\n')
        f.close()

if __name__ == '__main__':
    try:
        bot = botControl()

    except rospy.ROSInterruptException:
        pass
