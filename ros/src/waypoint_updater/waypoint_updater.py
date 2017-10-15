#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import TwistStamped

import math
import tf
import numpy as np
from speed_envelope import SpeedEnvelope
from copy import deepcopy

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MIN_RED_TL_DIST = 100 #Min distance to consider a RED traffic light to stop


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints = None
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        #next_red_tl_wp holds the wp id of the next traffic light detected on red.
        self.actual_v = 0
        self.next_red_tl_wp = None
        #To check if the red light state changed
        self.last_red_tl_wp = None
        self.upcoming_red_light_sub = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_callback,
                queue_size =1)


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        
        # Publisher for the current waypoint id
        self.waypoint_id_pub = rospy.Publisher('/current_waypoint_id', Int32, queue_size=1)

        self.cte_pub = rospy.Publisher('/current_cte', Float32, queue_size=1)

        # Make the closest point id callable in the class
        self.closest_point = 0

        #Init the SpeedEnvelope
        self.envelope = SpeedEnvelope(self.base_waypoints, safety_distance = 10)
        self.stop_at_red_wps = None
        self.red_tl_approach = False

        rospy.spin()

    def next_waypoint(self, x, y, yaw):
        # Find the closest waypoint
        dist = [(x - wp.pose.pose.position.x)**2 + (y - wp.pose.pose.position.y)**2 for wp in self.base_waypoints]

        # Find the closest waypoint in the last 'final_waypoints' list
        # dist = [(x - wp.pose.pose.position.x)**2 + (y - wp.pose.pose.position.y)**2 for wp in self.base_waypoints[self.closest_point:self.closest_point + LOOKAHEAD_WPS]]

        closest = np.argmin(dist)
        wp = self.base_waypoints[closest]

        # Is it in front of or behind the vehicle?
        angle = math.atan2(wp.pose.pose.position.y - y, wp.pose.pose.position.x - x)
        relative_angle = (angle - yaw) % (2 * math.pi)
        
        if (relative_angle > 0.5 * math.pi) & (relative_angle < 1.5 * math.pi):
            # Behind, so return next waypoint index
            self.closest_point = (closest + 1) % len(self.base_waypoints)
        else:
            # In front, so return this one
            self.closest_point = closest

        # Calculate the cte value based on the closest point and the current x, y
        wp_1 = self.base_waypoints[self.closest_point]
        wp_2 = self.base_waypoints[(self.closest_point + 1) % len(self.base_waypoints)]

        # Get the angle between the current point and the next waypoint
        angle1 = math.atan2(y - wp_1.pose.pose.position.y, x - wp_1.pose.pose.position.x)
        # Get the angle of the waypoint
        angle2 = math.atan2(wp_2.pose.pose.position.y - wp_1.pose.pose.position.y, wp_2.pose.pose.position.x - wp_1.pose.pose.position.x)
        

        CTE = math.sqrt((wp_1.pose.pose.position.y - y)**2 + (wp_1.pose.pose.position.x - x)**2) * math.sin(angle2 - angle1)
        self.cte_pub.publish(Float32(CTE))

        return self.closest_point

    def current_velocity_callback(self, current_velocity) :
        self.actual_v = current_velocity.twist.linear.x    


    def pose_cb(self, msg):        
        #rospy.logerr('pose_cb called')
        if not self.base_waypoints:
            rospy.logerr('Empty Waypoints in pose_cb')
            return
        _, _, yaw = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,
                                                              msg.pose.orientation.y,
                                                              msg.pose.orientation.z,
                                                              msg.pose.orientation.w])
        # Find the index of the next waypoint
        #rospy.logerr('Non-empty waypoints in pose_cb')
        next_wp_id = self.next_waypoint(msg.pose.position.x, msg.pose.position.y, yaw)
        #rospy.logerr('Next waypoint id = {0}'.format(next_wp_id))
        self.waypoint_id_pub.publish(Int32(next_wp_id))
        # Prepare a list of the upcoming waypoints
        upcoming_waypoints = [self.base_waypoints[idx % len(self.base_waypoints)]
                              for idx in range(next_wp_id, next_wp_id + LOOKAHEAD_WPS + 1)]
        # Prepare message
        msg_drive = Lane(waypoints=upcoming_waypoints)
        # Publish it
        
        if self.red_tl_approach:
            msg_red = Lane(waypoints = self.stop_at_red_wps)            
            self.final_waypoints_pub.publish(msg_red)            
        else:
            self.final_waypoints_pub.publish(msg_drive)            

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints
        # Unsubscribe from base waypoints to improve performance
        self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        rospy.logwarn('Next traffic light id = {0}'.format(msg))
        wp_id = msg.data
        self.last_red_tl_wp = self.next_red_tl_wp #Save previous value
        if wp_id == -1 :
            self.next_red_tl_wp = None
        else :
            self.next_red_tl_wp = wp_id
            #rospy.logwarn('Next RED traffic light val = {0}'.format(self.base_waypoints[wp_id].pose))
        
        # First implementation of speed envelope. 
        #We need to make sure that this gets created once, not every time a red is detected
        self.envelope.base_wps = self.base_waypoints

        if self.distance(self.base_waypoints, self.closest_point, self.next_red_tl_wp) < 100:
            self.stop_at_red_wps = self.envelope.get_envelope(self.closest_point, self.next_red_tl_wp, self.actual_v)
            self.red_tl_approach = True
            rospy.logwarn('red approach = {0}'.format(self.red_tl_approach))
        else:
            self.red_tl_approach = False
            rospy.logwarn('red approach = {0}'.format(self.red_tl_approach))    
        

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        if wp1 == None or wp2 == None: 
            return 1000 #Far away to be replaced by something safe
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
