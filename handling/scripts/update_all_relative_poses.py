#!/usr/bin/env python3

# uses the roslaunch API to enable all UR robots

import rospy
import roslaunch



class UpdateAllRelativePoses:
    
    def config(self):
        self.robot_names = rospy.get_param('~robot_names', ['mur620a', 'mur620b', 'mur620c', 'mur620d'])
        self.UR_prefixes = rospy.get_param('~UR_prefixes', ['UR10_l', 'UR10_r'])
        self.node_name = rospy.get_param('~node_name', 'update_relative_pose')
        self.launch_pkg = rospy.get_param('~launch_pkg', 'manipulator_control')


    def __init__(self):
        self.config()
        self.start_move_UR_to_home_pose()


    def start_move_UR_to_home_pose(self):
        for robot_name in self.robot_names:
            for UR_prefix in self.UR_prefixes:
                virtual_object_topic = "/virtual_object/object_pose"
                TCP_pose_topic = "/" + robot_name + "/" + UR_prefix + "/global_tcp_pose"
                relative_pose_topic = "/" + robot_name + "/" + UR_prefix + "/relative_pose"
                #namespace = "/" + robot_name + "/" + UR_prefix + "/"
                namespace = ""
                process = self.launch_ros_node(self.node_name, self.launch_pkg, self.node_name + '.py', namespace, '' , virtual_object_topic = virtual_object_topic, TCP_pose_topic = TCP_pose_topic, relative_pose_topic = relative_pose_topic)
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
    rospy.init_node('update_all_relative_poses')
    UpdateAllRelativePoses()
    rospy.spin()





