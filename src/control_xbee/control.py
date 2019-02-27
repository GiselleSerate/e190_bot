#!/usr/bin/env python
import rospy
import rospkg
from xbee import XBee
import serial
import tf
import math
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from e190_bot.msg import ir_sensor
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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
        # self.rate = rospy.Rate(5) # 5hz
        while not rospy.is_shutdown():
            self.odom_pub();
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
        self.last_encoder_measurementL = 0
        self.last_encoder_measurementR = 0
        self.diffEncoderL = 0
        self.diffEncoderR = 0
        self.bot_angle = 0

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
        LPWM = int(8.52*abs(LAvel)+6.36)
        RPWM = int(8.61*abs(RAvel)+6.08)

        if(LPWM < 15):
            LPWM = 0 # Floor left
        elif(LPWM > 255):
            LPWM = 255 # Ceiling left
        if(RPWM < 15):
            RPWM = 0 # Floor right
        elif(RPWM > 255):
            RPWM = 255 # Ceiling right

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
                update = self.xbee.wait_read_frame()
            except:
                pass

            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            # Encoder readings as radians: l, r
            # encoder_measurements = [x * math.pi / 720 for x in [data[-1], data[-2]]]
            # encoder_measurements = [x * math.pi / 720 for x in [data[-2], data[-1]]]
            # encoder_measurements = [x * math.pi / 720 for x in [data[3], data[4]]]
            # encoder_measurements = [x * math.pi / 720 for x in [data[4], data[3]]]
            # self.diffEncoderL = encoder_measurements[0] - self.last_encoder_measurementL
            # self.diffEncoderR = encoder_measurements[1] - self.last_encoder_measurementR

            encoder_measurementsL = data[4] * np.math.pi / 720.0 # TODO: Until we fix the Teensy code, compensate for switched encoders
            encoder_measurementsR = data[3] * np.math.pi / 720.0

            # Calculate how many radians the encoders have moved since the last odom measurement
            self.diffEncoderL = encoder_measurementsL - self.last_encoder_measurementL
            self.diffEncoderR = encoder_measurementsR - self.last_encoder_measurementR

            # Save last encoder measurements
            # self.last_encoder_measurementL = encoder_measurements[0]
            # self.last_encoder_measurementR = encoder_measurements[1]
            self.last_encoder_measurementL = encoder_measurementsL
            self.last_encoder_measurementR = encoder_measurementsR

            # If either measurement is old, reset encoder differences
            if(self.diffEncoderL > 3 or self.diffEncoderR > 3 or self.diffEncoderL < -3 or self.diffEncoderR < -3):
                self.diffEncoderL = 0
                self.diffEncoderR = 0

            # Calculate changes in theta and distance with respect to the robot frame
            del_theta = ((self.diffEncoderR - self.diffEncoderL) * self.wheel_radius)/(2.0 * self.bot_radius)
            del_s = ((self.diffEncoderR + self.diffEncoderL) * self.wheel_radius)/2.0

            self.bot_angle += del_theta/2.0
            self.bot_angle = self.bot_angle % (2*math.pi) # Loop

            # Update x and y with deltas
            self.Odom.pose.pose.position.x += del_s * math.cos(self.bot_angle)
            self.Odom.pose.pose.position.y += del_s * math.sin(self.bot_angle)
            self.Odom.pose.pose.position.z = .0
            quat = quaternion_from_euler(.0, .0, self.bot_angle)
            self.Odom.pose.pose.orientation.x = quat[0]
            self.Odom.pose.pose.orientation.y = quat[1]
            self.Odom.pose.pose.orientation.z = quat[2]
            self.Odom.pose.pose.orientation.w = quat[3]

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
            self.pubRangeSensor(range_measurements)

        if(self.data_logging):
            self.log_data();

        self.time = rospy.Time.now()

    def ir_cal(self, val):
        """Convert ir sensor data to meters. The effective range of our sensors
        is from .2m to 1.5m. The best fit at this range was a power curve."""
        if(val == 0): # Prevent division by 0
            return 0
        return 242.76 * (val ** -1.075)

    def pubRangeSensor(self, ranges):
        """Publish the sensor recordings"""
        self.ir_L.distance = self.ir_cal(ranges[1])
        self.ir_C.distance = self.ir_cal(ranges[0])
        self.ir_R.distance = self.ir_cal(ranges[2])

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