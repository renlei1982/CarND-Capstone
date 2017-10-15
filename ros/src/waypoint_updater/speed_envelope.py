
#! /usr/bin/env python
from styx_msgs.msg import Lane, Waypoint

import math

import numpy as np

class SpeedEnvelope(object):

    def __init__(self, base_wps, safety_distance=10):
        self.base_wps = base_wps
        self.safety_distance = safety_distance        

    def get_envelope(self, car_wp_id, red_light_wp_id, actual_v):
        if self.base_wps == None:
            return None
        v_mps = actual_v * 0.44704 # m/s
        # the descending speed curve will start at least at 10 m/s
        v_mps = min(5.0, v_mps)
        speed_envelope = [self.base_wps[i % len(self.base_wps)]
                              for i in range(car_wp_id, red_light_wp_id)]
        for wp in speed_envelope:
            s = self.distance(self.base_wps, car_wp_id, red_light_wp_id) - self.safety_distance
            s = min(s, 0.)         
            t_to_stop = s/v_mps if abs(v_mps) > 0. else 0.
            if t_to_stop > 1.0 :
                wp.twist.twist.linear.x = s/t_to_stop
            else:
                wp.twist.twist.linear.x = 0.
            speed_envelope[-1].twist.twist.linear.x = 0.
        return speed_envelope   


    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a,b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range (wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
    