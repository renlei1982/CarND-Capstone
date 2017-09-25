
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import rospy
from pid import PID
from yaw_controller import YawController


class Controller(object):
    def __init__(self):
        # TODO: Implement
        # Fixed parameters
        self.vehicle_mass = 0
        self.brake_deadband = 0
        self.wheel_radius = 0
        self.accel_limit = 0
        self.wheel_base = 0
        self.steer_ratio = 0
        self.max_lat_accel = 0
        self.max_steer_angle = 0

        # To be Updated in each cycle
        self.twist_command = None
        self.current_velocity = None
        self.enabled = True
        self.sample_time = 1/50 # initial value, gets updated in loop


        self.throttle_PID = PID(1, 0.0001, 0.1) # Dummy values
        self.throttle_error = 0

        #initial control values	
        self.throttle = 0
        self.brake = 0
        self.steer = 0

        #self.yaw_ctrl = YawController(self.wheel_base,
        #                              self.steer_ratio,
        #                              0,
        #                              self.max_lat_accel,
        #                              self.max_steer_angle) # Set the min_speed as 0



    def control(self, target_v, target_angular_v, actual_v, dbw_status):
        # TODO: Change the arg, kwarg list to suit your needs
        # If we drive slower than the target sppeed, we push the gas pedal (throttle), othwise not
        if target_v > actual_v-0.1:
            self.throttle_error = target_v - actual_v
            self.throttle = self.throttle_PID.step(self.throttle_error, self.sample_time)
            self.brake = 0
        else:
            self.throttle = 0
            if target_v < actual:
                self.brake = 0.5 # we need a much better controller. 
                #we may define a deceleration curve and define
                #stopping in front of the traffic light
                self.steer = 0

        # Return throttle, brake, steer
        return self.throttle, self.brake, self.steer
