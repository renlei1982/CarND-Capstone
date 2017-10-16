
#! /usr/bin/env python
from styx_msgs.msg import Lane, Waypoint

import math

import numpy as np
import rospy
class SpeedEnvelope(object):

    def __init__(self, base_wps, safety_distance=10):
        self.base_wps = base_wps
        self.safety_distance = safety_distance        
        self.envelope_distances = [10, 20, 35, 45, 60]

    def get_envelope(self, car_wp_id, red_light_wp_id, actual_v):
        if self.base_wps == None:
            return None
        v_mps = actual_v * 0.44704 # m/s
        # the descending speed curve will start at least at 10 m/s
        v_mps = min(5.0, v_mps)
        speed_envelope = [self.base_wps[i % len(self.base_wps)]
                              for i in range(car_wp_id, red_light_wp_id)]
        v_mps = actual_v * 0.44704
        wp_i = 0
        wp_speed = v_mps
        for wp in speed_envelope:
            dist_wp_to_rtl = self.distance(self.base_wps, car_wp_id + wp_i, red_light_wp_id) - self.safety_distance
            wp_i = wp_i + 1

            for dist in range(len(self.envelope_distances)) :
                if dist_wp_to_rtl < self.envelope_distances[dist] :
                    wp_speed = dist
                    break

            wp.twist.twist.linear.x = wp_speed #s/t_to_stop
            
            rospy.logwarn("dist:{0}, v:{1}, new v:{2}".format(dist_wp_to_rtl, v_mps, wp_speed))
            #rospy.logwarn("dist:{0}, v:{1}, t:{2}, new v:{3}".format(s, v_mps, t_to_stop, wp.twist.twist.linear.x))
        speed_envelope[-1].twist.twist.linear.x = 0
        return speed_envelope



    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a,b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range (wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
    