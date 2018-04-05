#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tensorflow as tflow
import tf
import cv2
import yaml
import math
import os

STATE_COUNT_THRESHOLD = 3
TL_DISTANCE_LIMIT = 150

class TLDetector(object):
    def __init__(self):

        rospy.init_node('tl_detector')
	print os.getcwd()
	print(os.path.dirname(os.path.realpath(__file__)))

	MODEL_NAME = 'light_classification/inferencemodel'

	self.model_path = MODEL_NAME + '/frozen_inference_graph.pb'

	PATH_TO_LABELS = 'light_classification/training_setup/object-detection.pbtxt'

        # Build the model
        self.detection_graph = tflow.Graph()
        # create config
        config = tflow.ConfigProto()

        # Create the graph
        with self.detection_graph.as_default():
            self.graph_def = tflow.GraphDef()
            with tflow.gfile.GFile(self.model_path, 'rb') as fid:
                serialized_graph = fid.read()
                self.graph_def.ParseFromString(serialized_graph)
                tflow.import_graph_def(self.graph_def, name='')
                rospy.loginfo('Loaded frozen tensorflow model: %s', self.model_path)

            # Create a reusable sesion attribute
            # self.sess = tflow.Session(graph=self.detection_graph, config=config)
        rospy.logwarn("TL detect: init")

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.has_wps = False
        self.prev_pose = None
        self.heading = -3 * math.pi
        
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        self.stop_lines = []
        
        for sl_pos in stop_line_positions:
            a_light = TrafficLight()
            a_light.pose = PoseStamped()
            a_light.pose.pose.position.x = sl_pos[0]
            a_light.pose.pose.position.y = sl_pos[1]
            a_light.pose.pose.position.z = 0
            self.stop_lines.append(a_light)

        update_rate = rospy.Rate(5)

        while not rospy.is_shutdown():
           update_rate.sleep()

    def pose_cb(self, msg):
        self.prev_pose = self.pose
        self.pose = msg
        if (self.prev_pose and self.pose and self.prev_pose != self.pose):
            this_heading = math.atan2(self.pose.pose.position.y - self.prev_pose.pose.position.y, self.pose.pose.position.x - self.prev_pose.pose.position.x)
            this_distance = math.hypot(self.pose.pose.position.y - self.prev_pose.pose.position.y, self.pose.pose.position.x - self.prev_pose.pose.position.x)
            
            if ( 0.01 < this_distance ) :
                self.heading = this_heading

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        self.has_wps = True
        rospy.loginfo("WP: " + str(len(self.waypoints.waypoints)))

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        # rospy.loginfo("PTL: " + str(light_wp) + " " + str(state))

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
            light_wp = light_wp if (state == TrafficLight.RED or TrafficLight.YELLOW == state) else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            
        self.state_count += 1

    def get_closest_waypoint(self, pose, all_waypoints):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
            all_waypoints : waypoints to search through

        Returns:
            int: index of the closest waypoint in all_waypoints
            distance to waypoint
            directed angle to waypoint

        """
        min_distance = 10000000000
        best_wp_index = -1
        
        if (0<len(all_waypoints)):
            for i,wp in enumerate(all_waypoints):
                this_distance = math.hypot(wp.pose.pose.position.x - pose.position.x, wp.pose.pose.position.y - pose.position.y)
                    
                if (this_distance < min_distance):
                    best_wp_index = i
                    min_distance = this_distance
        
        dir_angle = 0
        if (0<=best_wp_index):
            dir_angle = math.atan2(all_waypoints[best_wp_index].pose.pose.position.y - pose.position.y, 
                all_waypoints[best_wp_index].pose.pose.position.x - pose.position.x)
            
        return best_wp_index, min_distance, dir_angle

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

        #Deep Learning
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        height, width, channels = cv_image.shape
        
        #Get classification
        tl_result = self.light_classifier.get_classification(cv_image, self.detection_graph)
        
        #Computer Vision
        #cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        #height, width, channels = cv_image.shape
        
        #Get classification
        #tl_result = self.light_classifier.get_cv_classification(cv_image)
        return tl_result

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if (not self.has_wps):
            rospy.loginfo("no wps")
            return -1, TrafficLight.UNKNOWN
        
        light = None
        light_wp = -1
        tl_use = False
        
        if(self.pose):
            car_position, car_dist_wp, car_dir_wp = self.get_closest_waypoint(self.pose.pose, self.waypoints.waypoints)
            tl_position, car_dist_tl, car_dir_tl = self.get_closest_waypoint(self.pose.pose, self.lights)
                
            tl_angle = math.fabs(math.fmod(math.fabs(car_dir_tl-self.heading), 2*math.pi))
            tl_use = (tl_angle < math.pi/2 and car_dist_tl <= TL_DISTANCE_LIMIT)

        #TODO find the closest visible traffic light (if one exists)
		
        sim_tl_state = TrafficLight.UNKNOWN

        # this is debug path only
        if (-1<tl_position):
            if (tl_use):
                light = self.lights[tl_position]
                light_stop_line_position, _, _ = self.get_closest_waypoint(self.lights[tl_position].pose.pose, self.stop_lines)
                light_wp, _, _ = self.get_closest_waypoint(self.stop_lines[light_stop_line_position].pose.pose, self.waypoints.waypoints)
                
            sim_tl_state = self.lights[tl_position].state

        rospy.loginfo_throttle(5, "ptl car at wp " + str(car_position))
        
        if light:
            state = self.get_light_state(light)
            rospy.loginfo("ptl: c=" + str(state) + " sim" + str(sim_tl_state))
            
            return light_wp, state
 
        
        # found nothing
        return -1, TrafficLight.UNKNOWN
        
if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
