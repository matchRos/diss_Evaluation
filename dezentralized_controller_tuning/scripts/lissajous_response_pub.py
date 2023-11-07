#!/usr/bin/env python3

# this node creates a lissajous curve and publishes it as a target pose for the formation controller
# it also publishes the resulting target velocity for the formation controller

import rospy

from geometry_msgs.msg import Pose, Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf import transformations, broadcaster
import math
from copy import deepcopy

class LissajousResponsePublisher:
    
    def config(self):
        self.Ax = rospy.get_param("~Ax", 2.0)
        self.Ay = rospy.get_param("~Ay", 2.0)
        self.omega_x = rospy.get_param("~omega_x", 1.0)
        self.omega_y = rospy.get_param("~omega_y", 2.0)
        self.delta_x = rospy.get_param("~delta_x", 0.0)
        self.delta_y = rospy.get_param("~delta_y", 90) * math.pi/180.0
        self.velocity = rospy.get_param("~velocity", 0.1)
        self.orientation_offset = rospy.get_param("~orientation_offset", 45.0) * math.pi/180.0 
        #self.number_of_points = rospy.get_param("~number_of_points", 1000)
        self.distance_between_points = rospy.get_param("~distance_between_points", 0.05)
        self.lissajous_path_topic = rospy.get_param("~lissajous_path_topic", "/lissajous_path")
        self.virtual_leader_pose_topic = rospy.get_param("~virtual_leader_pose_topic", "/virtual_leader/leader_pose")
        pass
    
    def __init__(self):
        self.config()
        self.path_publisher = rospy.Publisher(self.lissajous_path_topic, Path, queue_size=10, latch=True)
        rospy.Subscriber(self.virtual_leader_pose_topic, PoseStamped, self.virtual_leader_pose_callback)
        
    def run(self):
        
        # create the path message
        self.path = Path()
        self.path.header.frame_id = "map"
        self.path.header.stamp = rospy.Time.now()
        self.path.poses = []
        self.pose_stamped = PoseStamped()
        self.pose_stamped.header.stamp = rospy.Time.now()
        self.pose_stamped.header.frame_id = "map"
        
        
        # compute the lissajous curve considering the leader's current position and orientation
        rospy.wait_for_message(self.virtual_leader_pose_topic, PoseStamped)
        rospy.loginfo("Received virtual leader pose")
        virtual_leader_angle = transformations.euler_from_quaternion([self.virtual_leader_pose.pose.orientation.x, self.virtual_leader_pose.pose.orientation.y, self.virtual_leader_pose.pose.orientation.z, self.virtual_leader_pose.pose.orientation.w])[2]
        for t in range(0, 1000):
            self.pose_stamped.pose.position.x = self.virtual_leader_pose.pose.position.x + self.Ax * math.sin(self.omega_x * t/100.0) * math.cos(virtual_leader_angle) - self.Ay * math.sin(self.omega_y * t/100.0) * math.sin(virtual_leader_angle)
            self.pose_stamped.pose.position.y = self.virtual_leader_pose.pose.position.y + self.Ax * math.sin(self.omega_x * t/100.0) * math.sin(virtual_leader_angle) + self.Ay * math.sin(self.omega_y * t/100.0) * math.cos(virtual_leader_angle)
            self.pose_stamped.pose.position.z = 0.0
            self.pose_stamped.pose.orientation.x = 0.0
            self.pose_stamped.pose.orientation.y = 0.0
            self.pose_stamped.pose.orientation.z = 0.0
            self.pose_stamped.pose.orientation.w = 1.0
            self.path.poses.append(deepcopy(self.pose_stamped))
            
        self.path_publisher.publish(self.path)
        rospy.sleep(1.0)
                
    def virtual_leader_pose_callback(self, msg):
        self.virtual_leader_pose = msg
        pass
        
        
if  __name__ == "__main__":
    rospy.init_node("lissajous_response_publisher")
    lissajous_response_publisher = LissajousResponsePublisher()
    lissajous_response_publisher.run()
    rospy.spin()
