#!/usr/bin/env python3

# send a trigger to each "zero_FT_sensor" service to zero the FT sensor

import rospy
import roslaunch
from std_srvs.srv import Trigger, TriggerRequest


class ZeroAllFTSensors:
    
    def config(self):
        self.robot_names = rospy.get_param('~robot_names', ['mur620a','mur620b','mur620c', 'mur620d'])
        self.UR_prefixes = rospy.get_param('~UR_prefixes', ['UR10_l', 'UR10_r'])


    def __init__(self):
        self.config()
        self.zero_FT_sensors()

    def zero_FT_sensors(self):
        for robot_name in self.robot_names:
            for UR_prefix in self.UR_prefixes:
                trigger_client = rospy.ServiceProxy(robot_name + "/" + UR_prefix + "/ur_hardware_interface/zero_ftsensor", Trigger)
                rospy.loginfo("Waiting for " + robot_name + "/" + UR_prefix + "/ur_hardware_interface/zero_ftsensor")
                trigger_client.wait_for_service()
                rospy.loginfo("Zeroing FT sensor for " + robot_name + "/" + UR_prefix)
                trigger_request = TriggerRequest()
                trigger_client(trigger_request)
                rospy.loginfo("FT sensor zeroed for " + robot_name + "/" + UR_prefix)

        # shutdown node
        rospy.signal_shutdown('All FT sensors zeroed')







if __name__ == '__main__':
    rospy.init_node('zero_all_FT_sensors')
    ZeroAllFTSensors()
    rospy.spin()





