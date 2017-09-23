
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import rospy
from pid import PID
from yaw_controller import YawController


class Controller(object):
    def __init__(self, vehicle_mass, wheel_radius, brake_deadband,
                 decel_limit, accel_limit, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        # Fixed parameters
        self.vehicle_mass = vehicle_mass
        self.brake_deadband = brake_deadband
        self.wheel_radius = wheel_radius
        self.accel_limit = accel_limit
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        
        # To be Updated in each cycle
        self.twist_command = None
        self.current_velocity = None
        self.enabled = False

        self.throttle_PID = PID(0.1, 0.1, 0.1) # Dummy values
        




    def control(self, target_v, target_angular_v, actual_v, dbw_status):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
