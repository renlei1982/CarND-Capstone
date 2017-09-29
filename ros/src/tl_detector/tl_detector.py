#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import numpy as np

STATE_COUNT_THRESHOLD = 3
LIGHT_HORIZON = 100 # How many waypoints ahead to look for light

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.current_waypoint_id = None
        self.light_waypoints = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_waypoint_id', Int32, self.current_waypoint_cb)
        self.base_waypoints_sub = sub2

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        rospy.loginfo('Loading traffic light classifier from {0}'.format(rospy.get_param('~path')))
        self.light_classifier = TLClassifier(rospy.get_param('~path'))
        self.listener = tf.TransformListener()

        self.enabled = rospy.get_param('~enabled')
        if not self.enabled:
            rospy.logwarn('Traffic light detection is disabled')

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        # Unsubscribe from base waypoints to improve performance
        self.base_waypoints_sub.unregister()
        # Work out the closest waypoint to each traffic light
        for light in self.config['light_positions']:
            light_wp = self.get_closest_waypoint(light[0], light[1])
            # Store waypoint range in which light will be visible to car
            self.light_waypoints.append(((light_wp - LIGHT_HORIZON) % len(self.waypoints), light_wp))

    def current_waypoint_cb(self, msg):
        # Save current waypoint id published from waypoint updater
        self.current_waypoint_id = msg.data

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        if not self.enabled:
            return
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
        Args:
            x, y: traffic light location

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        dist = [(x - wp.pose.pose.position.x)**2 + (y - wp.pose.pose.position.y)**2 for wp in self.waypoints]
        closest = np.argmin(dist)
        return closest

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        car_position = self.current_waypoint_id
        if (not car_position) | (not self.light_waypoints):
            rospy.logwarn('Data not ready for light classification')
            return -1, TrafficLight.UNKNOWN

        # Find the closest visible traffic light (if one exists)
        light = None
        for l in self.light_waypoints:
            visible = (car_position >= l[0]) & (car_position <= l[1])
            if visible:
                light = l[1]
                break

        if light:
            state = self.get_light_state(light)
            rospy.loginfo('Light at waypoint {0} classified as {1}'.format(light, state))
            return light, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
