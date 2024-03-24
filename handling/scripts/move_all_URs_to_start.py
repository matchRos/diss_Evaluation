#!/usr/bin/env python3

# uses the roslaunch API to start the move_UR_to_home_pose node for every UR robot

import rospy
import roslaunch



class MoveAllURsToStart:
    
    def config(self):
        self.robot_names = rospy.get_param('~robot_names', ['mur620a', 'mur620d'])
        self.UR_prefixes = rospy.get_param('~UR_prefixes', ['UR10_l', 'UR10_r'])
        self.node_name = rospy.get_param('~node_name', 'move_UR_to_home_position')
        self.launch_pkg = rospy.get_param('~launch_pkg', 'ur_utilities')
        self.target_position_name = rospy.get_param('~target_position_name', 'handling_position')
        self.move_group_names = rospy.get_param('~move_group_names', ['UR_arm_l', 'UR_arm_r'])

    def __init__(self):
        self.config()
        self.start_move_UR_to_home_pose()


    def start_move_UR_to_home_pose(self):
        for robot_name in self.robot_names:
            for UR_prefix in self.UR_prefixes:
                process = self.launch_ros_node(self.node_name, self.launch_pkg, 'move_UR_to_home_position.py', "/" + robot_name, "" , UR_prefix=UR_prefix, tf_prefix=robot_name, home_position = self.target_position_name, move_group_name = self.move_group_names[self.UR_prefixes.index(UR_prefix)])
                # check if the node is still running
                while process.is_alive() and not rospy.is_shutdown():
                    rospy.sleep(1)

        # shutdown node
        rospy.signal_shutdown('All UR robots moved to start position')


    def launch_ros_node(self,node_name, package_name, node_executable, namespace="/", node_args="", **params):
        # get param names from kwargs
        param_names = params.keys()
        # set params on param server
        for param_name in param_names:
            rospy.set_param(namespace + node_name + "/" + param_name, params[param_name])

        package = package_name
        executable = node_executable
        name = node_name
        node = roslaunch.core.Node(package=package, node_type=executable, name=name, namespace=namespace,
                                        machine_name=None, args=node_args, output="screen")
        
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node)
        return process




if __name__ == '__main__':
    rospy.init_node('move_all_URs_to_start')
    MoveAllURsToStart()
    rospy.spin()





