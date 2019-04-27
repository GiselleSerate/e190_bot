
from E160_state import *
from E160_PF import *
import math
import datetime

class E160_robot:

    def __init__(self, environment, address, robot_id):
        self.environment = environment
        self.state_est = E160_state()
        self.state_est.set_state(0,0,0)
        self.state_des = E160_state()
        self.state_des.set_state(0,0,0)
        self.state_draw = E160_state()
        self.state_draw.set_state(0,0,0)
        self.state_odo = E160_state()
        self.state_odo.set_state(0,0,0) # real position for simulation

        #self.v = 0.05
        #self.w = 0.1
        self.R = 0
        self.L = 0
        self.radius = 0.147 / 2
        self.width = 2*self.radius
        self.wheel_radius = 0.03
        self.address = address
        self.ID = self.address.encode().__str__()[-1]
        self.last_measurements = []
        self.robot_id = robot_id
        self.manual_control_left_motor = 0
        self.manual_control_right_motor = 0
        self.file_name = 'Log/Bot' + str(self.robot_id) + '_' + datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S') + '.txt'
        self.make_headers()
        self.encoder_resolution = 1440

        self.last_encoder_measurements = [0,0]
        self.encoder_measurements = [0,0]
        self.range_measurements = [0,0,0]
        self.last_simulated_encoder_R = 0
        self.last_simulated_encoder_L = 0

        self.Kpho = 1#1.0
        self.Kalpha = 2#2.0
        self.Kbeta = -0.5#-0.5
        self.max_velocity = 0.05
        self.point_tracked = True
        self.encoder_per_sec_to_rad_per_sec = 10

        self.PF = E160_PF(environment, self.width, self.wheel_radius, self.encoder_resolution)


    def update(self, deltaT):

        # # get sensor measurements
        # self.encoder_measurements, self.range_measurements = self.update_sensor_measurements(deltaT)
        #
        # # update odometry
        # delta_s, delta_theta = self.update_odometry(self.encoder_measurements)
        #
        # # update simulated real position, find ground truth for simulation
        # self.state_odo = self.localize(self.state_odo, delta_s, delta_theta, self.range_measurements)

        # localize with particle filter
        self.state_est = self.PF.LocalizeEstWithParticleFilter(self.encoder_measurements, self.range_measurements)

        # to out put the true location for display purposes only.
        self.state_draw = self.state_odo

        # call motion planner
        # self.motion_planner.update_plan()

        # determine new control signals
        self.R, self.L = self.update_control(self.range_measurements)

        # send the control measurements to the robot
        self.send_control(self.R, self.L, deltaT)

        #print(self.state_est.x, self.state_est.y, self.state_est.t)
        # self.state_est.set_state(0,0,0)


    def update_sensor_measurements(self, deltaT):

        if self.environment.robot_mode == "HARDWARE MODE":
            command = '$S @'
            self.environment.xbee.tx(dest_addr = self.address, data = command)

            update = self.environment.xbee.wait_read_frame()

            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            encoder_measurements = data[-2:]
            range_measurements = data[:-2]

        elif self.environment.robot_mode == "SIMULATION MODE":
            encoder_measurements = self.simulate_encoders(self.R, self.L, deltaT)
            sensor1 = self.simulate_range_finder(self.state_odo, self.PF.sensor_orientation[0])
            sensor2 = self.simulate_range_finder(self.state_odo, self.PF.sensor_orientation[1])
            sensor3 = self.simulate_range_finder(self.state_odo, self.PF.sensor_orientation[2])
            range_measurements = [sensor1, sensor2, sensor3]

        return encoder_measurements, range_measurements



    def localize(self, state_est, delta_s, delta_theta, range_measurements):
        state_est = self.update_state(state_est, delta_s, delta_theta)

        return state_est

    def angle_wrap(self, a):
        while a > math.pi:
            a = a - 2*math.pi
        while a < -math.pi:
            a = a + 2*math.pi

        return a


    def update_control(self, range_measurements):

        if self.environment.control_mode == "MANUAL CONTROL MODE":
            desiredWheelSpeedR = self.manual_control_right_motor
            desiredWheelSpeedL = self.manual_control_left_motor

        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":
            desiredWheelSpeedR, desiredWheelSpeedL = self.point_tracker_control()

        return desiredWheelSpeedR, desiredWheelSpeedL



    def point_tracker_control(self):

        # If the desired point is not tracked yet, then track it
        if not self.point_tracked:


            ############ Student code goes here ############################################


            pass
        # the desired point has been tracked, so don't move
        else:
            desiredWheelSpeedR = 0
            desiredWheelSpeedL = 0

        return desiredWheelSpeedR,desiredWheelSpeedL


    def send_control(self, R, L, deltaT):

        # send to actual robot !!!!!!!!
        if self.environment.robot_mode == "HARDWARE MODE":
            if (L < 0):
                LDIR = 0
            else:
                LDIR = 1

            if (R < 0):
                RDIR = 0
            else:
                RDIR = 1
            RPWM = int(abs(R))
            LPWM = int(abs(L))

            command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            self.environment.xbee.tx(dest_addr = self.address, data = command)



    def simulate_encoders(self, R, L, deltaT):
        right_encoder_measurement = -int(R*self.encoder_per_sec_to_rad_per_sec*deltaT) + self.last_simulated_encoder_R
        left_encoder_measurement = -int(L*self.encoder_per_sec_to_rad_per_sec*deltaT) + self.last_simulated_encoder_L
        self.last_simulated_encoder_R = right_encoder_measurement
        self.last_simulated_encoder_L = left_encoder_measurement

        #print "simulate_encoders", R, L, right_encoder_measurement, left_encoder_measurement
        return [left_encoder_measurement, right_encoder_measurement]

    def simulate_range_finder(self, state, sensorT):
        '''Simulate range readings, given a simulated ground truth state'''
        p = self.PF.Particle(state.x, state.y, state.theta, 0)

        return self.PF.FindMinWallDistance(p, self.environment.walls, sensorT)

    def make_headers(self):
        f = open(self.file_name, 'a+')
        f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} \n'.format('R1', 'R2', 'R3', 'RW', 'LW'))
        f.close()



    def log_data(self):
        f = open(self.file_name, 'a+')

        # edit this line to have data logging of the data you care about
        data = [str(x) for x in [1,2,3,4,5]]

        f.write(' '.join(data) + '\n')
        f.close()


    def set_manual_control_motors(self, R, L):

        self.manual_control_right_motor = int(R*256/100)
        self.manual_control_left_motor = int(L*256/100)



    def update_odometry(self, encoder_measurements):

        delta_s = 0
        delta_theta = 0

         # ****************** Additional Student Code: Start ************


        # Calculate difference in movement from last time step
        diffEncoder0 = +(encoder_measurements[0]-self.last_encoder_measurements[0]);
        diffEncoder1 = -(encoder_measurements[1]-self.last_encoder_measurements[1]);

        # At the first iteration, zero out
        if abs(diffEncoder0)> 1000 or abs(diffEncoder1)> 1000:
            diffEncoder0 = 0
            diffEncoder1 = 0

        #Localization
        wheelDistanceL = - 2 * 3.14 * self.wheel_radius / self.encoder_resolution * (diffEncoder0); # Negative since this is left wheel

        wheelDistanceR = + 2 * 3.14 * self.wheel_radius / self.encoder_resolution * (diffEncoder1); # Positive since this is right wheel


        # print wheelDistanceL
        # remember for next time
        self.last_encoder_measurements[0] = encoder_measurements[0];
        self.last_encoder_measurements[1] = encoder_measurements[1];

        # Calculate v x dt and w x dt
        delta_s = 0.5 * (wheelDistanceR + wheelDistanceL);
        delta_theta = 0.5 / self.radius * (wheelDistanceR - wheelDistanceL);


        # ****************** Additional Student Code: End ************

        # keep this to return appropriate changes in distance, angle
        return delta_s, delta_theta




    def update_state(self, state, delta_s, delta_theta):

          # ****************** Additional Student Code: Start ************
        state.x = state.x + delta_s*math.cos(state.theta+delta_theta/2)
        state.y = state.y + delta_s*math.sin(state.theta+delta_theta/2)
        state.theta = state.theta + delta_theta


        # ****************** Additional Student Code: End ************

        # keep this to return the updated state
        return state
