#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
import math
#from jmt import JerkMinimizingTrajectory
from rospy import get_param
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
MAX_DECEL = 1.0
BRAKE_DISTANCE = 50
STOP_WAYPOINT_OFFSET = 5 # Number of waypoints to stop before the actual traffic light.

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.waypoints_received = False
        self.current_velocity = 0

        self.stop_node = None

        self.stop_jmt = None
        self.max_velocity = (get_param('/waypoint_loader/velocity') * 1000.) / (60. * 60.) # velocity in mps

        rospy.spin()

    def wrap_wp_index(self, idx):
        number_of_waypoints = len(self.waypoints)
        while idx < 0:
            idx += number_of_waypoints
        return idx % number_of_waypoints
        
    def velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x
        
    def pose_cb(self, msg):

        if not self.waypoints_received : return # Don't do anything if waypoints are not present

        # Query the KD Tree. nearest_waypoint will be an index of the waypoint closest to the car
        _, nearest_waypoint = self.waypoint_tree.query([msg.pose.position.x, msg.pose.position.y])

        waypoint_list = []
        stop_waypoint = -1
        
        #TODO decelerate waypoints
        if self.stop_node >= 0:
            stop_waypoint = self.wrap_wp_index(self.stop_node - STOP_WAYPOINT_OFFSET)

            if self.stop_jmt is None:

                rospy.loginfo('Decelerating to waypoint {}'.format(stop_waypoint))
                self.stop_jmt = True
                
                

                self.set_waypoint_velocity(self.waypoints, stop_waypoint, 0)

                #TODO Wrap wayopints
                #TODO implement JMT

                stop_distance = self.distance(self.waypoints, nearest_waypoint, stop_waypoint)
                brake_distance = min(BRAKE_DISTANCE, stop_distance)


                for i in range(nearest_waypoint, stop_waypoint):
                    dist = self.distance(self.waypoints, i, stop_waypoint)
                    if dist < brake_distance:
                        # Decelerate linear
                        decel_velocity = max(min(self.current_velocity, self.max_velocity), 3.)
                        self.set_waypoint_velocity(self.waypoints, i, dist/brake_distance*decel_velocity)

        else:
            self.stop_jmt = None
            for i in range(nearest_waypoint, nearest_waypoint+LOOKAHEAD_WPS):
                self.set_waypoint_velocity(self.waypoints, i, self.max_velocity)

        number_of_waypoints = len(self.waypoints)
        for idx in range(nearest_waypoint, nearest_waypoint+LOOKAHEAD_WPS):
            
            if stop_waypoint >= 0 and self.wrap_wp_index(idx) > self.stop_node: continue 
            
            waypoint_list.append(self.waypoints[self.wrap_wp_index(idx)])

        lane = Lane()
        lane.waypoints = waypoint_list

        self.final_waypoints_pub.publish(lane)


    def waypoints_cb(self, lane):
        rospy.loginfo("Received {} waypoints".format(len(lane.waypoints)))

        self.waypoints = lane.waypoints

        xy_waypoint_list = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.waypoints]
        self.waypoint_tree = KDTree(xy_waypoint_list)

        self.waypoints_received = True

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        if self.stop_node != msg.data:
            self.stop_jmt = None
            self.stop_node = msg.data
            rospy.loginfo("Received STOP waypoint: {}".format(msg.data))


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[self.wrap_wp_index(waypoint)].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[self.wrap_wp_index(wp1)].pose.pose.position, waypoints[self.wrap_wp_index(i)].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
