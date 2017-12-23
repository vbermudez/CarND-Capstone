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
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Waypoint, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Waypoint, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.px = None
        self.py = None
        self.yaw = None
        self.current_waypoint_idx = None
        self.target_velocity = 10.0

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.px = msg.pose.position.x
        self.py = msg.pose.position.y
        orientation = msg.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.yaw = tf.transformations.euler_from_quaternion(q)
    
    def get_dist(self, wp):
        wx = wp.pose.pose.position.x
        wy = wp.pose.pose.position.y
        dx = wx - self.px
        dy = wy - self.py
        return dx, dy

    def transform_2_local(self, wp):
        dx, dy = self.get_dist(wp)
        local_wx = math.cos(-self.yaw) * dx - math.sin(-self.yaw) * dy
        local_wy = math.sin(-self.yaw) * dx - math.cos(-self.yaw) * dy
        return local_wx, local_wy, math.atan2(local_wy, local_wx)

    def is_wp_ahead(self, wp):
        wx, wy, phi = self.transform_2_local(wp)
        return wx > .0

    def dist_2_wp(self, wp):
        dx, dy = self.get_dist(wp)
        return math.sqrt(dx * dx + dy * dy)

    def find_closest_wp(self, msg):
        """
            Finds the closest waypoint to the vehicle position.
        """
        min_dist = 1e9
        min_idx = None
        for idx,wp in enumerate(msg.waypoints):
            dist = self.dist_2_wp(wp)
            if dist < min_dist:
                min_dist = dist
                min_idx = idx
        num_wp = len(msg.waypoints)
        closest_idx = min_idx
        closest_wp = msg.waypoints[closest_idx]
        if not self.is_wp_ahead(closest_wp):
            closest_idx (closest_idx + 1) % num_wp
        return closest_idx

    def waypoints_cb(self, waypoints):
        if self.px is None:
            return
        lane = Lane()
        num_wp = len(waypoints.waypoints)
        current_idx = self.find_closest_wp(waypoints)
        for i in range(LOOKAHEAD_WPS):
            wp = waypoints.waypoints[current_idx]
            nwp = Waypoint()
            nwp.pose = wp.pose
            nwp.twist.twist.linear.x = self.target_velocity
            lane.waypoints.append(nwp)
            current_idx = (current_idx + 1) % num_wp
        self.final_waypoints_pub.publish(lane)

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
