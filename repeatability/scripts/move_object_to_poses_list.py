#!/usr/bin/env python3

# moves the object to several different poses to measure the repeatability of the handling task 
# poses are specified in the poses_list parameter

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose
from tf import transformations

class MoveObjectToInitialPose:

    def poses_list_config(self):

        self.poses_list = rospy.get_param("~poses_list", [])

        for i in range(self.repetitions):
            # offset = [0.0, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0]
            # self.offset_and_add_poses(self.initial_pose, offset,i)
            # offset = [0.0, 0.0 , 0.1, 0.0, 0.0, 0.0, 0.0]
            # self.offset_and_add_poses(self.initial_pose, offset,i)
            # offset = [0.1, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0]
            # self.offset_and_add_poses(self.initial_pose, offset,i)
            offset = [0.1, 0.1 , 0.1, 0.0, 0.0, 0.0, 0.0]
            self.offset_and_add_poses(self.initial_pose, offset,i)
            offset = [-0.2, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0]
            self.offset_and_add_poses(self.initial_pose, offset,i)
            offset = [0.0, -0.2 , 0.0, 0.0, 0.0, 0.0, 0.0]
            self.offset_and_add_poses(self.initial_pose, offset,i)
            # offset = [0.0, 0.0 , -0.2, 0.0, 0.0, 0.0, 0.0]
            # self.offset_and_add_poses(self.initial_pose, offset,i)
            # offset = [0.2, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0]
            # self.offset_and_add_poses(self.initial_pose, offset,i)
            # offset = [0.0, 0.2 , 0.0, 0.0, 0.0, 0.0, 0.0]
            # self.offset_and_add_poses(self.initial_pose, offset,i)

            
        # print("Poses list: ", self.poses_list)
        # print("Number of unique poses: ", self.number_of_unique_poses)

    def offset_and_add_poses(self, pose, offset,iteration):
        pose_out = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        pose_out[0] = pose[0] + offset[0]
        pose_out[1] = pose[1] + offset[1]
        pose_out[2] = pose[2] + offset[2]
        q1 = transformations.quaternion_multiply([pose[3], pose[4], pose[5], pose[6]], transformations.quaternion_from_euler(offset[3], offset[4], offset[5]))
        pose_out[3] = q1[0]
        pose_out[4] = q1[1]
        pose_out[5] = q1[2]
        pose_out[6] = q1[3]
        self.poses_list.append(pose_out)

        if iteration == 0:
            self.number_of_unique_poses += 1

    def config(self):
        self.initial_pose = [36.99933238093275,36.63378957278167,1.3265077455433467,-0.00039379802380474864,0.0032322841463110884,-0.007039548363270753,-0.9999699205581911]
        self.repetitions = rospy.get_param("~repetitions", 30)
        self.currecnt_pose_topic = rospy.get_param("~current_pose_topic", "/virtual_object/object_pose")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/virtual_object/object_cmd_vel")
        self.actual_object_pose_topic = rospy.get_param("~actual_object_pose_topic", "/qualisys/match_tuch/pose")
        self.max_velocity = rospy.get_param("~velocity", 0.07)
        self.max_acceleration = rospy.get_param("~acceleration", 0.010)
        self.Kp_linear = rospy.get_param("~Kp_linear", 1.0)
        self.Kp_angular = rospy.get_param("~Kp_angular", 0.5)
        self.rate = rospy.get_param("~rate", 100)
        self.target_tolerance = rospy.get_param("~target_tolerance", 0.002)
        self.object_pose_filter_gain = rospy.get_param("~object_pose_filter_gain", 0.03)
        self.poses_list_config()
        pass

    def __init__(self):
        self.number_of_unique_poses = 0
        self.config()
        self.x_position_array = [[0.0] * self.repetitions for i in range(self.number_of_unique_poses)]
        self.y_position_array = [[0.0] * self.repetitions for i in range(self.number_of_unique_poses)]
        self.z_position_array = [[0.0] * self.repetitions for i in range(self.number_of_unique_poses)]
        self.x_orientation_array = [[0.0] * self.repetitions for i in range(self.number_of_unique_poses)]
        self.y_orientation_array = [[0.0] * self.repetitions for i in range(self.number_of_unique_poses)]
        self.z_orientation_array = [[0.0] * self.repetitions for i in range(self.number_of_unique_poses)]
        self.w_orientation_array = [[0.0] * self.repetitions for i in range(self.number_of_unique_poses)]
        self.poses_measured_couter = 0
        self.current_repetition = 0
        self.cmd_publisher = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        rospy.Subscriber(self.currecnt_pose_topic, PoseStamped, self.pose_callback)
        rospy.Subscriber(self.actual_object_pose_topic, PoseStamped, self.mocap_callback)
        self.object_pose_filtered = Pose()
        self.control_output_old = Twist()
        rospy.sleep(1)
        self.next_pose = True
        self.run()

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # get current target pose
            self.compute_current_target()
            # compute the error
            error = self.compute_error()
            # compute the control input
            control_output = self.compute_control_output(error)
            # publish the control input
            self.cmd_publisher.publish(control_output)
            rate.sleep()

    def compute_current_target(self):

        if self.next_pose == True:
            if len(self.poses_list) == 0:
                rospy.signal_shutdown("No more poses to reach")    
            else:
                self.current_target = self.poses_list.pop(0)
                self.next_pose = False


    def compute_error(self):
        error = Twist()
        error.linear.x =  self.current_target[0] - self.current_pose.position.x 
        error.linear.y =  self.current_target[1] - self.current_pose.position.y 
        error.linear.z =  self.current_target[2] - self.current_pose.position.z 
        q_error = transformations.quaternion_multiply([self.current_target[3], self.current_target[4], self.current_target[5], self.current_target[6]], transformations.quaternion_inverse([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w]) )
         #transformations.quaternion_multiply([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w], transformations.quaternion_inverse(self.initial_pose[3:]))
        euler_error = transformations.euler_from_quaternion(q_error)
        error.angular.x = euler_error[0]
        error.angular.y = euler_error[1]
        error.angular.z = euler_error[2]
        
        return error
    
    def compute_control_output(self, error):
        control_output = Twist()
        control_output.linear.x = self.Kp_linear * error.linear.x
        control_output.linear.y = self.Kp_linear * error.linear.y
        control_output.linear.z = self.Kp_linear * error.linear.z
        control_output.angular.x = self.Kp_angular * error.angular.x
        control_output.angular.y = self.Kp_angular * error.angular.y
        control_output.angular.z = self.Kp_angular * error.angular.z

        # limit acceleration
        control_output.linear.x = self.control_output_old.linear.x + self.max_acceleration *  (control_output.linear.x - self.control_output_old.linear.x) / abs(control_output.linear.x - self.control_output_old.linear.x) if abs(control_output.linear.x - self.control_output_old.linear.x) > self.max_acceleration else control_output.linear.x
        control_output.linear.y = self.control_output_old.linear.y + self.max_acceleration *  (control_output.linear.y - self.control_output_old.linear.y) / abs(control_output.linear.y - self.control_output_old.linear.y) if abs(control_output.linear.y - self.control_output_old.linear.y) > self.max_acceleration else control_output.linear.y
        control_output.linear.z = self.control_output_old.linear.z + self.max_acceleration *  (control_output.linear.z - self.control_output_old.linear.z) / abs(control_output.linear.z - self.control_output_old.linear.z) if abs(control_output.linear.z - self.control_output_old.linear.z) > self.max_acceleration else control_output.linear.z
        control_output.angular.x = self.control_output_old.angular.x + self.max_acceleration *  (control_output.angular.x - self.control_output_old.angular.x) / abs(control_output.angular.x - self.control_output_old.angular.x) if abs(control_output.angular.x - self.control_output_old.angular.x) > self.max_acceleration else control_output.angular.x
        control_output.angular.y = self.control_output_old.angular.y + self.max_acceleration *  (control_output.angular.y - self.control_output_old.angular.y) / abs(control_output.angular.y - self.control_output_old.angular.y) if abs(control_output.angular.y - self.control_output_old.angular.y) > self.max_acceleration else control_output.angular.y
        control_output.angular.z = self.control_output_old.angular.z + self.max_acceleration *  (control_output.angular.z - self.control_output_old.angular.z) / abs(control_output.angular.z - self.control_output_old.angular.z) if abs(control_output.angular.z - self.control_output_old.angular.z) > self.max_acceleration else control_output.angular.z

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


        # check if target is reached
        if abs(error.linear.x) < self.target_tolerance and abs(error.linear.y) < self.target_tolerance and abs(error.linear.z) < self.target_tolerance and abs(error.angular.x) < self.target_tolerance and abs(error.angular.y) < self.target_tolerance and abs(error.angular.z) < self.target_tolerance:
            self.next_pose = True
            self.measure_object_pose()

        return control_output
    
    def measure_object_pose(self):
        # stop the object
        control_output = Twist()
        self.cmd_publisher.publish(control_output)
        # wait for the object to stop and for the mocap to stabilize (average the mocap pose for 3 second)
        rospy.sleep(3)
        # measure the object pose
        #print("Measured object pose: ", self.object_pose_filtered)

        # save the measured pose to pose array 
        self.x_position_array[self.poses_measured_couter][self.current_repetition] = self.object_pose_filtered.position.x
        self.y_position_array[self.poses_measured_couter][self.current_repetition] = self.object_pose_filtered.position.y
        self.z_position_array[self.poses_measured_couter][self.current_repetition] = self.object_pose_filtered.position.z
        self.x_orientation_array[self.poses_measured_couter][self.current_repetition] = self.object_pose_filtered.orientation.x
        self.y_orientation_array[self.poses_measured_couter][self.current_repetition] = self.object_pose_filtered.orientation.y
        self.z_orientation_array[self.poses_measured_couter][self.current_repetition] = self.object_pose_filtered.orientation.z
        self.w_orientation_array[self.poses_measured_couter][self.current_repetition] = self.object_pose_filtered.orientation.w



        self.poses_measured_couter += 1
        if self.poses_measured_couter == self.number_of_unique_poses:
            self.poses_measured_couter = 0
            self.current_repetition += 1

            print("X position array: ", self.x_position_array)
            print("Y position array: ", self.y_position_array)
            print("Z position array: ", self.z_position_array)
            print("X orientation array: ", self.x_orientation_array)
            print("Y orientation array: ", self.y_orientation_array)
            print("Z orientation array: ", self.z_orientation_array)
            print("W orientation array: ", self.w_orientation_array)
            print("\n")

            # if self.current_repetition == self.repetitions:
            #     rospy.signal_shutdown("All poses measured")

    
    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def mocap_callback(self, msg):
        self.actual_object_pose = msg.pose
        self.object_pose_filtered.position.x = self.object_pose_filtered.position.x + self.object_pose_filter_gain * (self.actual_object_pose.position.x - self.object_pose_filtered.position.x)
        self.object_pose_filtered.position.y = self.object_pose_filtered.position.y + self.object_pose_filter_gain * (self.actual_object_pose.position.y - self.object_pose_filtered.position.y)
        self.object_pose_filtered.position.z = self.object_pose_filtered.position.z + self.object_pose_filter_gain * (self.actual_object_pose.position.z - self.object_pose_filtered.position.z)
        q1 = self.object_pose_filtered.orientation.x + self.object_pose_filter_gain * (self.actual_object_pose.orientation.x - self.object_pose_filtered.orientation.x)
        q2 = self.object_pose_filtered.orientation.y + self.object_pose_filter_gain * (self.actual_object_pose.orientation.y - self.object_pose_filtered.orientation.y)
        q3 = self.object_pose_filtered.orientation.z + self.object_pose_filter_gain * (self.actual_object_pose.orientation.z - self.object_pose_filtered.orientation.z)
        q4 = self.object_pose_filtered.orientation.w + self.object_pose_filter_gain * (self.actual_object_pose.orientation.w - self.object_pose_filtered.orientation.w)
        # normalize the quaternion
        norm = (q1**2 + q2**2 + q3**2 + q4**2)**0.5
        self.object_pose_filtered.orientation.x = q1/norm
        self.object_pose_filtered.orientation.y = q2/norm
        self.object_pose_filtered.orientation.z = q3/norm
        self.object_pose_filtered.orientation.w = q4/norm

        # print("Filtered object pose: ", self.object_pose_filtered)
        # print("Actual object pose: ", self.actual_object_pose)

if __name__ == '__main__':
    rospy.init_node('move_object_to_initial_pose')
    MoveObjectToInitialPose()
    rospy.spin()

