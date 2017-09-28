#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        # TODO: Create `TwistController` object
        self.controller = Controller()

        self.controller.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        self.controller.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.controller.brake_deadband = rospy.get_param('~brake_deadband', .1)
        self.controller.decel_limit = rospy.get_param('~decel_limit', -5)
        self.controller.accel_limit = rospy.get_param('~accel_limit', 1.)
        self.controller.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.controller.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        self.controller.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        self.controller.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        self.controller.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Set up the sample time point for loop and PID control
        self.start_time = 0.0

        #Set up the yaw_angle for the yaw_controller
        self.yaw_angle = 0.0

        # Set up the current_velocity
        self.current_velocity = 0.0

        #Set up the dbw_enable
        self.dbw_enable = True

        # TODO: Arguments to be specified for TwistController

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_callback)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_callback)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_callback)

        self.loop()

    def twist_cmd_callback(self, twist_command):
        #self.controller.twist_command = twist_command
        self.yaw_angle = twist_command.twist.angular.z

    def current_velocity_callback(self, current_velocity):
        #self.controller.current_velocity = current_velocity
        self.current_velocity = current_velocity.twist.linear.x

    def dbw_enabled_callback(self, dbw_enabled):
        #self.controller.enable = True #dbw_enabled;
        self.dbw_enable = True


    def loop(self):

        rate = rospy.Rate(50) # 50Hz

        while not self.start_time:
            self.start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled

            self.controller.sample_time = rospy.Time.now().to_sec() - self.start_time
            self.start_time = rospy.Time.now().to_sec()
            throttle, brake, steer = self.controller.control(10.0, self.yaw_angle, self.current_velocity, True)

            #if <dbw is enabled>:

            self.publish(throttle, brake, steer)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
