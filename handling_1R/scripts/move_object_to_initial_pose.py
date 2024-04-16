#!/usr/bin/env python3

# moves the object back to the initial pose after the handling task by publishing a twist message to the object_cmd_vel topic

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose
from tf import transformations, TransformBroadcaster

class MoveObjectToInitialPose:
    def config(self):
        self.initial_pose = rospy.get_param("~initial_pose", [36.17285072743418, 33.27203882369522, 0.9746758171588863 + 0.1, 0.9353907628485877, 0.35360382292714954, 0.002602042566780646, -0.001298677528486306])
        self.currecnt_pose_topic = rospy.get_param("~current_pose_topic", "/virtual_object/object_pose")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/virtual_object/object_cmd_vel")
        self.max_velocity = rospy.get_param("~velocity", 0.05)
        self.Kp_linear = rospy.get_param("~Kp_linear", 0.2)
        self.Kp_angular = rospy.get_param("~Kp_angular", 0.05)
        self.rate = rospy.get_param("~rate", 100)
        self.tf_broadcaster = TransformBroadcaster()
        pass

    def __init__(self):
        self.config()
        self.cmd_publisher = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        rospy.Subscriber(self.currecnt_pose_topic, PoseStamped, self.pose_callback)
        rospy.sleep(1)
        self.run()

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # compute the error
            error = self.compute_error()
            # compute the control input
            control_output = self.compute_control_output(error)
            # publish the control input
            self.cmd_publisher.publish(control_output)
            # publish the tf
            self.publish_tf()
            rate.sleep()

    def publish_tf(self):
        #self.tf_broadcaster.sendTransform((self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z), (self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w), rospy.Time.now(), "object", "world")
        self.tf_broadcaster.sendTransform((self.initial_pose[0], self.initial_pose[1], self.initial_pose[2]), (self.initial_pose[3], self.initial_pose[4], self.initial_pose[5], self.initial_pose[6]), rospy.Time.now(), "initial_object", "map")


    def compute_error(self):
        error = Twist()
        error.linear.x =  self.initial_pose[0] - self.current_pose.position.x 
        error.linear.y =  self.initial_pose[1] - self.current_pose.position.y 
        error.linear.z =  self.initial_pose[2] - self.current_pose.position.z 
        q_error = transformations.quaternion_multiply([self.initial_pose[3], self.initial_pose[4], self.initial_pose[5], self.initial_pose[6]], transformations.quaternion_inverse([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w]) )
         #transformations.quaternion_multiply([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w], transformations.quaternion_inverse(self.initial_pose[3:]))
        euler_error = transformations.euler_from_quaternion(q_error)
        error.angular.x = euler_error[0] * 1.0
        error.angular.y = euler_error[1] * -1.0
        error.angular.z = euler_error[2] * -1.0
        print("Error: ", error)

        return error
    
    def compute_control_output(self, error):
        control_output = Twist()
        control_output.linear.x = self.Kp_linear * error.linear.x
        control_output.linear.y = self.Kp_linear * error.linear.y
        control_output.linear.z = self.Kp_linear * error.linear.z
        control_output.angular.x = self.Kp_angular * error.angular.x
        control_output.angular.y = self.Kp_angular * error.angular.y
        control_output.angular.z = self.Kp_angular * error.angular.z

        # limit the control input
        if abs(control_output.linear.x) > self.max_velocity:
            control_output.linear.x = self.max_velocity if control_output.linear.x > 0 else -self.max_velocity
        if abs(control_output.linear.y) > self.max_velocity:
            control_output.linear.y = self.max_velocity if control_output.linear.y > 0 else -self.max_velocity
        if abs(control_output.linear.z) > self.max_velocity:
            control_output.linear.z = self.max_velocity if control_output.linear.z > 0 else -self.max_velocity
        if abs(control_output.angular.x) > self.max_velocity:
            control_output.angular.x = self.max_velocity if control_output.angular.x > 0 else -self.max_velocity
        if abs(control_output.angular.y) > self.max_velocity:
            control_output.angular.y = self.max_velocity if control_output.angular.y > 0 else -self.max_velocity
        if abs(control_output.angular.z) > self.max_velocity:
            control_output.angular.z = self.max_velocity if control_output.angular.z > 0 else -self.max_velocity

        #print("Control output: ", control_output)
        return control_output
    

    
    def pose_callback(self, msg):
        self.current_pose = msg.pose


if __name__ == '__main__':
    rospy.init_node('move_object_to_initial_pose')
    MoveObjectToInitialPose()
    rospy.spin()

