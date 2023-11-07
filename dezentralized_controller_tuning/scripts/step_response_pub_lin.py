#!/usr/bin/env python3

# this node publishes a step response in cmd_vel to evaluate and tune the formation controller

import rospy
from geometry_msgs.msg import Twist

class StepResponsePublisherLin:
    
    def config(self):
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/virtual_leader/cmd_vel")
    
    def __init__(self):
        self.config()
        self.publisher = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.twist = Twist()

    
    def run(self):
        # get current time
        t0 = rospy.Time.now()
        # wait for 1 second
        self.twist.linear.x = 0.1
        while rospy.Time.now() - t0 < rospy.Duration(5.0):
            self.publisher.publish(self.twist)
            rospy.sleep(0.01)
        # wait for another second
        self.twist.linear.x = 0.2
        while rospy.Time.now() - t0 < rospy.Duration(10.0):
            self.publisher.publish(self.twist)
            rospy.sleep(0.01)
        # stop the robot
        self.twist.linear.x = 0.0
        self.publisher.publish(self.twist)
    
    
    
if  __name__ == "__main__":
    rospy.init_node("step_response_publisher")
    step_response_publisher = StepResponsePublisherLin()
    step_response_publisher.run()
    rospy.spin()