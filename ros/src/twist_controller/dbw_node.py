#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Float32
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
        ctr_params = {
                "vehicle_mass" : rospy.get_param('~vehicle_mass', 1736.35),
                "fuel_capacity" : rospy.get_param('~fuel_capacity', 13.5),
                "brake_deadband" : rospy.get_param('~brake_deadband', .1),
                "decel_limit" : rospy.get_param('~decel_limit', -5),
                "accel_limit" : rospy.get_param('~accel_limit', 1.),
                "wheel_radius" : rospy.get_param('~wheel_radius', 0.2413),
                "wheel_base" : rospy.get_param('~wheel_base', 2.8498),
                "steer_ratio" : rospy.get_param('~steer_ratio', 14.8),
                "max_lat_accel" : rospy.get_param('~max_lat_accel', 3.),
                "max_steer_angle" : rospy.get_param('~max_steer_angle', 8.)
                }


        self.controller = Controller(**ctr_params)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.max_speed = 0.0
        # Set up the sample time point for loop and PID control
        self.start_time = 0.0

        #Set up the yaw_angle for the yaw_controller
        self.yaw_angle = 0.0
        # init actual speed read
        self.actual_v = 0

        #init cte value
        self.cte_value = 0
        
        self.enabled = True


        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_callback, queue_size = 1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_callback,
                queue_size =1)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_callback)
        #rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb)
        rospy.Subscriber('/current_cte', Float32, self.cte_callback)
        

        self.loop()

    def twist_cmd_callback(self, twist_command):
        self.controller.twist_command = twist_command
        self.yaw_angle = twist_command.twist.angular.z
        #I guess this can be used to set the max allowable speed, not sure though
        #to be used in self.controller.control(...)
        self.max_speed = twist_command.twist.linear.x

    def current_velocity_callback(self, current_velocity):
        self.controller.current_velocity = current_velocity #maybe unnecessary
        self.actual_v = current_velocity.twist.linear.x

    def dbw_enabled_callback(self, dbw_enabled):
        self.controller.enabled = dbw_enabled.data
        self.enabled = dbw_enabled.data

    #def final_waypoints_cb(self, msg):
        #self.target_v = msg.waypoints[0].twist.twist.linear.x

    def cte_callback(self, msg):
        self.cte_value = msg.data


    def loop(self):

        rate = rospy.Rate(10) # 50Hz

        while not self.start_time:
            self.start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled

            self.controller.sample_time = rospy.Time.now().to_sec() - self.start_time
            self.start_time = rospy.Time.now().to_sec()
            throttle, brake, steer = self.controller.control(self.max_speed, self.yaw_angle,
                    self.actual_v, self.cte_value, True)

            if self.enabled:
                self.publish(throttle, brake, steer)
            else:
                self.controller.speed_PID.reset()

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        rospy.loginfo(throttle)
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
