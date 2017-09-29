#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import tf
import numpy as np

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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints = None
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # Publisher for the current waypoint id
        self.waypoint_id_pub = rospy.Publisher('/current_waypoint_id', Int32, queue_size=1)

        rospy.spin()

    def next_waypoint(self, x, y, yaw):
        # Find the closest waypoint
        dist = [(x - wp.pose.pose.position.x)**2 + (y - wp.pose.pose.position.y)**2 for wp in self.base_waypoints]
        closest = np.argmin(dist)
        wp = self.base_waypoints[closest]
        # Is it in front of or behind the vehicle?
        angle = math.atan2(wp.pose.pose.position.y - y, wp.pose.pose.position.x - x)
        relative_angle = (angle - yaw) % (2 * math.pi)
        if (relative_angle > 0.5 * math.pi) & (relative_angle < 1.5 * math.pi):
            # Behind, so return next waypoint index
            return closest + 1 % len(self.base_waypoints)
        else:
            # In front, so return this one
            return closest

    def pose_cb(self, msg):
        if not self.base_waypoints:
            return
        _, _, yaw = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,
                                                              msg.pose.orientation.y,
                                                              msg.pose.orientation.z,
                                                              msg.pose.orientation.w])
        # Find the index of the next waypoint
        next_wp_id = self.next_waypoint(msg.pose.position.x, msg.pose.position.y, yaw)
        rospy.loginfo('Next waypoint id = {0}'.format(next_wp_id))
        self.waypoint_id_pub.publish(Int32(next_wp_id))
        # Prepare a list of the upcoming waypoints
        upcoming_waypoints = [self.base_waypoints[idx % len(self.base_waypoints)]
                              for idx in range(next_wp_id, next_wp_id + LOOKAHEAD_WPS + 1)]
        # Prepare message
        msg = Lane(waypoints=upcoming_waypoints)
        # Publish it
        self.final_waypoints_pub.publish(msg)

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints
        # Unsubscribe from base waypoints to improve performance
        self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
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
