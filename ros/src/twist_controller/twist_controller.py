
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import rospy
from pid import PID
from yaw_controller import YawController


class Controller(object):
    def __init__(self):
        # TODO: Implement
        # Fixed parameters
        self.vehicle_mass = None
        self.brake_deadband = None
        self.wheel_radius = None
        self.accel_limit = None
        self.wheel_base = None
        self.steer_ratio = None
        self.max_lat_accel = None
        self.max_steer_angle = None
        
        # To be Updated in each cycle
        self.twist_command = None
        self.current_velocity = None
        self.enabled = False
        self.sample_time = 1/50 # initial value, gets updated in loop

        self.throttle_PID = PID(0.1, 0.1, 0.1) # Dummy values

        self.throttle_error = 0

        self.yaw_ctrl = YawController(self.wheel_base,
                                      self.steer_ratio,
                                      0,
                                      self.max_lat_accel,
                                      self.max_steer_angle) # Set the min_speed as 0
        




    def control(self, target_v, target_angular_v, actual_v, dbw_status):
        # TODO: Change the arg, kwarg list to suit your needs

        throttle = self.throttle_PID.step(self.throttle_error, self.sample_time) #sample_time should be sampled from 'dbw_node.py' loop func 

        steer = self.yaw_ctrl.get_steering(target_v, target_angular_v, actual_v)

        brake = 0; # to be implemented
        # Return throttle, brake, steer
        return throttle, brake, steer
