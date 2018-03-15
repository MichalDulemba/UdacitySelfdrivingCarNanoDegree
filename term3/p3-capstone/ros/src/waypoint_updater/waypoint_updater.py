#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf

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
        rospy.init_node('waypoint_updater') #, log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Add other member variables you need below
        self.base_waypoints = None
        self.current_pose = None

        rospy.loginfo("Starting WaypointUpdater...")
        # 10 Hz frequency
        rate = rospy.Rate(10)
        # Loop until ROS master is running
        while not rospy.is_shutdown():
            # Processing
            self.process_loop()
            # Return control to ROS
            rate.sleep()

    def process_loop(self):
        if self.is_data_received():
            # Publish next waypoints
            self.publish_next_waypoints()
        return
    
    def is_data_received(self):
        return self.base_waypoints and self.current_pose
        
    def publish_next_waypoints(self):
        lane = Lane()
        # Current date/time
        lane.header.stamp = rospy.Time().now()
        # Exactly like in other nodes
        lane.header.frame_id = '/world'
        
        pose = self.current_pose.pose
        next_wp_idx = self.next_waypoint(pose)
        rospy.logdebug("Next WP Index {}".format(next_wp_idx))
                       
        num_base_points = len(self.base_waypoints)
        
        final_waypoints = [self.base_waypoints[p] for p in [idx % num_base_points for idx in range(next_wp_idx, next_wp_idx + LOOKAHEAD_WPS)]]
        # Temporary set constant speed
        speed = 2.78 #ms -> 10kph
        for idx in range(len(final_waypoints)):
            self.set_waypoint_velocity(final_waypoints, idx, speed)
            
        lane.waypoints = final_waypoints
        
        rospy.logdebug("Publishing {} waypoints...".format(len(final_waypoints)))
        
        # Publish 
        self.final_waypoints_pub.publish(lane)
    
    ''' 
    Find the closest waypoint index to position
    '''
    def closest_waypoint_idx(self, position):
        return min(xrange(len(self.base_waypoints)), key = lambda idx: self.euclidean_distance(position, self.base_waypoints[idx].pose.pose.position))
    
    ''' Return Euclidean distance '''
    def euclidean_distance(self, a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
    
    '''
    Find the closest waypoint ahead the position
    '''
    def next_waypoint(self, pose):
        wp_idx = self.closest_waypoint_idx(pose.position)
        closest_wp = self.base_waypoints[wp_idx]
        
        # Closest point
        wp_x = closest_wp.pose.pose.position.x
        wp_y = closest_wp.pose.pose.position.y
        
        # Current position
        x, y, yaw = self.get_position(pose)
        
        # Evaluate localization in car coordinates
        loc_x = (wp_x - x) * math.cos(yaw) + (wp_y - y) * math.sin(yaw)
        # If localization is negative, the point is behind, not ahead
        if loc_x < 0.0:
            wp_idx = wp_idx + 1
            
        return wp_idx
    
    '''
    Getting x,y and yaw from pose
    '''
    def get_position(self, pose):
        x = pose.orientation.x
        y = pose.orientation.y
        z = pose.orientation.z
        w = pose.orientation.w
        euler = tf.transformations.euler_from_quaternion([x, y, z, w])
        return [pose.position.x, pose.position.y, euler[2]]
        
    def pose_cb(self, pose):
        self.current_pose = pose
        rospy.logdebug("Received pose")

    def waypoints_cb(self, lane):
        self.base_waypoints = lane.waypoints
        rospy.logdebug("Received {} waypoints".format(len(self.base_waypoints)))

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
