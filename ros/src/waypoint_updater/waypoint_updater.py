#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import TwistStamped

import math
import tf
import numpy as np

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
MAX_DECEL = 1.0  # Max decelaration a



class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints = None
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        #next_red_tl_wp holds the wp id of the next traffic light detected on red.
        self.actual_v = 0
        self.next_red_tl_wp = -1
        #To check if the red light state changed
        self.last_red_tl_wp = -1
        self.upcoming_red_light_sub = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_callback, queue_size =1)


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        
        # Publisher for the current waypoint id
        self.waypoint_id_pub = rospy.Publisher('/current_waypoint_id', Int32, queue_size=1)

        self.cte_pub = rospy.Publisher('/current_cte', Float32, queue_size=1)

        # Make the closest point id callable in the class
        self.closest_point = 0

        self.stop_at_red_wps = None
        # self.red_tl_approach = False


        self.velocity = rospy.get_param('/waypoint_loader/velocity')

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


    # Deceleration control waypoint x speed calculation
    def decelerate(self, next_wp_id, next_tl_wp_id, waypoints):
        to_tl_steps = next_tl_wp_id - next_wp_id

        for wp_seq in range(to_tl_steps - 5, to_tl_steps + 1):
            self.set_waypoint_velocity(waypoints, wp_seq, -5.0)
            
        for wp_seq in range(to_tl_steps - 5):
            dist = self.distance(waypoints, wp_seq, to_tl_steps - 5)
            vel = math.sqrt(2 * MAX_DECEL * dist)/3.0
            if vel < 1.:
                vel = 0.
            self.set_waypoint_velocity(waypoints, wp_seq, min(vel, self.velocity))
            # wp.twist.twist.linear.x = min(vel, 0.0)
        return waypoints


    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    '''
    def distance_2(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)
    '''    

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
        # rospy.logerr('Non-empty waypoints in pose_cb')
        next_wp_id = self.next_waypoint(msg.pose.position.x, msg.pose.position.y, yaw)
        # Computing wp speeds, using closest wp
        
        # rospy.loginfo('Next waypoint id = {0}'.format(next_wp_id))

        self.waypoint_id_pub.publish(Int32(next_wp_id))
        # Prepare a list of the upcoming waypoints
        upcoming_waypoints = [self.base_waypoints[idx % len(self.base_waypoints)]
                              for idx in range(next_wp_id, next_wp_id + LOOKAHEAD_WPS + 1)]
        
        for wp in upcoming_waypoints:
            wp.twist.twist.linear.x = self.velocity * 0.27778

        # If the red light ahead is detected and within the range of 200 waypoints,
        # the x speed of the upcoming waypoits should be decelerated
        if (self.next_red_tl_wp - next_wp_id < 350 and self.next_red_tl_wp - next_wp_id > 50) or (self.next_red_tl_wp != None and self.next_red_tl_wp - next_wp_id < 50 and self.next_red_tl_wp > next_wp_id):
            upcoming_waypoints = self.decelerate(next_wp_id, self.next_red_tl_wp, upcoming_waypoints)

        # Prepare message
        self.final_waypoints_pub.publish(Lane(waypoints=upcoming_waypoints))


    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints
        # Unsubscribe from base waypoints to improve performance
        self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # rospy.logwarn('Next traffic light id = {0}'.format(msg))
        wp_id = msg.data
        self.last_red_tl_wp = self.next_red_tl_wp #Save previous value
        self.next_red_tl_wp = wp_id #Get if of most recent red tl found

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
