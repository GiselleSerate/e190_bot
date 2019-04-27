import math
import random
import numpy as np
import copy
from E160_state import*
from scipy.stats import norm


class E160_PF:

    def __init__(self, environment, robotWidth, wheel_radius, encoder_resolution):
        self.particles = []
        self.environment = environment
        self.numParticles = 400
        
        # maybe should just pass in a robot class?
        self.robotWidth = robotWidth
        self.radius = robotWidth/2
        self.wheel_radius = wheel_radius
        self.encoder_resolution = encoder_resolution
        self.FAR_READING = 1000
        
        # PF parameters
        self.IR_sigma = 0.2 # Range finder s.d
        self.odom_xy_sigma = 1.25   # odometry delta_s s.d
        self.odom_heading_sigma = 0.75  # odometry heading s.d
        self.particle_weight_sum = 0

        # define the sensor orientations
        self.sensor_orientation = [-math.pi/2, 0, math.pi/2] # orientations of the sensors on robot
        self.walls = self.environment.walls

        # initialize the current state
        self.state = E160_state()
        self.state.set_state(0,0,0)

        # TODO: change this later
        self.map_maxX = 1.0
        self.map_minX = -1.0
        self.map_maxY = 1.0
        self.map_minY = -1.0
        self.InitializeParticles()
        self.last_encoder_measurements =[0,0]

    def InitializeParticles(self):
        ''' Populate self.particles with random Particle 
            Args:
                None
            Return:
                None'''
        self.particles = []
        for i in range(0, self.numParticles):
            #self.SetRandomStartPos(i)
            self.SetKnownStartPos(i)

            
    def SetRandomStartPos(self, i):
        i.x = random.random() * (self.map_maxX - self.map_minX) + self.map_minX
        i.y = random.random() * (self.map_maxY - self.map_minY) + self.map_minY

    def SetKnownStartPos(self, i):
        i.x = self.state.x
        i.y = self.state.y
        i.theta = self.state.theta
            
    def LocalizeEstWithParticleFilter(self, delta_s, delta_heading, sensor_readings):
        ''' Localize the robot with particle filters. Call everything
            Args: 
                delta_s (float): change in distance as calculated by odometry
                delta_heading (float): change in heading as calcualted by odometry
                sensor_readings([float, float, float]): sensor readings from range fingers
            Return:
                None''' 
        for particle in self.particles:
            self.Propagate(delta_s, delta_heading, particle)
            self.CalculateWeight()
        
        
        return self.GetEstimatedPos()

    def Propagate(self, delta_s, delta_heading, i):
        '''Propagate all the particles from the last state with odometry readings
            Args:
                delta_s (float): distance traveled based on odometry
                delta_heading(float): change in heading based on odometry
            return:
                nothing'''
        # previous signature: def Propagate(self, encoder_measurements, i):

        i.x += delta_s * math.cos(self.heading + delta_heading / 2.0)
        i.y += delta_s * math.sin(self.heading + delta_heading / 2.0)

        i.heading += delta_heading
        i.heading = i.heading % (2 * math.pi)


    def CalculateWeight(self, sensor_readings, walls, particle):
        '''Calculate the weight of a particular particle
            Args:
                particle (E160_Particle): a given particle
                sensor_readings ( [float, ...] ): readings from the IR sesnors
                walls ([ [four doubles], ...] ): positions of the walls from environment, 
                            represented as 4 doubles 
            return:
                new weight of the particle (float) '''

        newWeight = 0
        # add student code here 
        particle.weight = newWeight
        # TODO see slides; Gaussian variance
        # end student code here
        return newWeight

    def Resample(self):
        '''Resample the particles systematically
            Args:
                None
            Return:
                None'''
        # add student code here 
        
        # TODO resample particles, preferring higher weights
        # do NOT condense
        
        # end student code here
        



    def GetEstimatedPos(self):
        ''' Calculate the mean of the particles and return it 
            Args:
                None
            Return:
                None'''
        # add student code here 
        
        # TODO average all particle states to get estimate and return it
        
        # end student code here
        
        return self.state


    def FindMinWallDistance(self, particle, walls, sensorT):
        ''' Given a particle position, walls, and a sensor, find 
            shortest distance to the wall
            Args:
                particle (E160_Particle): a particle 
                walls ([E160_wall, ...]): represents endpoint of the wall 
                sensorT: orientation of the sensor on the robot
            Return:
                distance to the closest wall' (float)'''
        # add student code here 
        
        # TODO for loop over sensors and get min 
            # leverage Bresneham to find first coordinate where you collide sensor with wall
            # and then get the distance to it in meters

        # end student code here
        
        return 0
    

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



