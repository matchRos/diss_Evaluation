#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf
from math import pi

class MoveMirToStartPose():

    def config(self):
        self.target_pose = rospy.get_param("~target_pose", [36,36,pi])
        self.tf_prefix = rospy.get_param("~tf_prefix", "mur620b")

    def __init__(self):
        rospy.init_node('move_mir_to_start_pose', anonymous=True)
        self.config()
    

    def send_target_pose(self):
        
        
        # Create a publisher to send the target pose
        target_pose_pub = rospy.Publisher("/" + self.tf_prefix + '/move_base_simple/goal', PoseStamped, queue_size=10)
        
        # Wait for the action server to become available
        rospy.loginfo('Waiting for move base action server')
        move_base_client = actionlib.SimpleActionClient("/" + self.tf_prefix + '/move_base_flex/move_base', MoveBaseAction)
        move_base_client.wait_for_server()
        rospy.loginfo('Move base action server is available')
        
        # Create a target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.pose.position.x = self.target_pose[0]
        target_pose.pose.position.y = self.target_pose[1]
        q = tf.transformations.quaternion_from_euler(0, 0, self.target_pose[2])
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]
        
        # Publish the target pose
        target_pose_pub.publish(target_pose)
        
        # Create a move base goal
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose = target_pose
        
        # Send the move base goal
        move_base_client.send_goal(move_base_goal)
        
        # Wait for the robot to reach the target pose
        move_base_client.wait_for_result()
        
        # Print the result of the action
        result = move_base_client.get_result()
        rospy.loginfo('Move base result: %s', result)

if __name__ == '__main__':
    try:
        MoveMirToStartPose().send_target_pose()
    except rospy.ROSInterruptException:
        pass
