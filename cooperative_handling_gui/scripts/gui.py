import sys
import subprocess
import time
import yaml
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QCheckBox, QLabel
from PyQt5.QtCore import QTimer
import threading

class ROSGui(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Multi-Robot Demo")
        self.setGeometry(100, 100, 400, 450)
        self.workspace_name = "catkin_ws_recker"
        self.layout = QVBoxLayout()

        # Status display for controllers
        self.status_label = QLabel("Controller Status: Not Checked")
        self.status_label.setStyleSheet("border: 1px solid black; padding: 5px;")
        self.layout.addWidget(self.status_label)
        
        # Timer for periodic status check
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.start_status_update)
        self.status_timer.start(3000)  # Check status every 3 seconds


        # Layout for robot checkboxes and UR checkboxes side by side
        robot_ur_layout = QHBoxLayout()

        # Left column: Robot checkboxes
        robot_layout = QVBoxLayout()
        self.robots = {
            "mur620a": QCheckBox("mur620a"),
            "mur620b": QCheckBox("mur620b"),
            "mur620c": QCheckBox("mur620c"),
            "mur620d": QCheckBox("mur620d"),
        }
        for checkbox in self.robots.values():
            robot_layout.addWidget(checkbox)

        # Right column: UR checkboxes
        ur_layout = QVBoxLayout()
        ur_layout.addWidget(QLabel("Select URs:"))  # Section label
        self.ur10_l = QCheckBox("UR10_l")
        self.ur10_r = QCheckBox("UR10_r")

        # Set UR checkboxes to be selected by default
        self.ur10_l.setChecked(True)
        self.ur10_r.setChecked(True)

        ur_layout.addWidget(self.ur10_l)
        ur_layout.addWidget(self.ur10_r)

        # Add both layouts to the horizontal layout
        robot_ur_layout.addLayout(robot_layout)
        robot_ur_layout.addLayout(ur_layout)

        # Buttons for ROS actions
        self.btn_virtual_leader = QPushButton("Start Virtual Leader")
        self.btn_virtual_leader.clicked.connect(lambda: self.launch_ros("virtual_leader", "virtual_leader.launch"))

        self.btn_virtual_object = QPushButton("Start Virtual Object")
        self.btn_virtual_object.clicked.connect(lambda: self.launch_ros("virtual_object", "virtual_object.launch"))

        self.btn_compute_center = QPushButton("Start Compute Object Center")
        self.btn_compute_center.clicked.connect(self.run_compute_object_center)

        self.btn_launch_drivers = QPushButton("Launch Drivers")
        self.btn_launch_drivers.clicked.connect(self.launch_drivers)

        self.btn_quit_drivers = QPushButton("Quit Drivers")
        self.btn_quit_drivers.clicked.connect(self.quit_drivers)

        self.btn_check_status = QPushButton("Check Status")
        self.btn_check_status.clicked.connect(self.update_status)
        self.layout.addWidget(self.btn_check_status)


        self.btn_zero_ft_sensors = QPushButton("Zero F/T Sensors")
        self.btn_zero_ft_sensors.clicked.connect(self.zero_ft_sensors)

        self.btn_turn_on_wrench = QPushButton("Turn on Wrench Controllers")
        self.btn_turn_on_wrench.clicked.connect(self.turn_on_wrench_controllers)

        self.btn_turn_on_arm = QPushButton("Turn on Arm Controllers")
        self.btn_turn_on_arm.clicked.connect(self.turn_on_arm_controllers)

        self.btn_turn_on_twist = QPushButton("Turn on Twist Controllers")
        self.btn_turn_on_twist.clicked.connect(self.turn_on_twist_controllers)

        self.btn_enable_all_urs = QPushButton("Enable all URs")
        self.btn_enable_all_urs.clicked.connect(self.enable_all_urs)

        self.btn_update_ur_relative = QPushButton("Update UR relative to object")
        self.btn_update_ur_relative.clicked.connect(self.update_ur_relative_to_object)

        # Buttons for Initial Pose (left & right)
        move_pose_layout = QHBoxLayout()
        self.btn_move_left = QPushButton("Move to Initial Pose Left")
        self.btn_move_left.clicked.connect(lambda: self.move_to_initial_pose("UR10_l"))
        self.btn_move_right = QPushButton("Move to Initial Pose Right")
        self.btn_move_right.clicked.connect(lambda: self.move_to_initial_pose("UR10_r"))

        move_pose_layout.addWidget(self.btn_move_left)
        move_pose_layout.addWidget(self.btn_move_right)

        # Layout for the cooperative admittance controller button and its checkbox
        admittance_layout = QHBoxLayout()

        self.btn_coop_admittance = QPushButton("Turn on cooperative Admittance Controller")
        self.btn_coop_admittance.setStyleSheet("background-color: orange; color: black; font-weight: bold;")  # Highlight button
        self.btn_coop_admittance.clicked.connect(self.turn_on_coop_admittance_controller)

        self.check_set_reference = QCheckBox("Set reference at runtime")
        self.check_set_reference.setChecked(True)  # Pre-select checkbox

        admittance_layout.addWidget(self.btn_coop_admittance)
        admittance_layout.addWidget(self.check_set_reference)

        # Add elements to the main layout
        self.layout.addLayout(robot_ur_layout)  # Robot & UR checkboxes
        self.layout.addWidget(self.btn_virtual_leader)
        self.layout.addWidget(self.btn_virtual_object)
        self.layout.addWidget(self.btn_compute_center)
        self.layout.addWidget(self.btn_launch_drivers)
        self.layout.addWidget(self.btn_quit_drivers)
        self.layout.addWidget(self.btn_zero_ft_sensors)
        self.layout.addLayout(move_pose_layout)  # Move buttons
        self.layout.addWidget(self.btn_turn_on_wrench)
        self.layout.addWidget(self.btn_turn_on_arm)
        self.layout.addWidget(self.btn_turn_on_twist)
        self.layout.addWidget(self.btn_enable_all_urs)
        self.layout.addWidget(self.btn_update_ur_relative)
        self.layout.addLayout(admittance_layout)  # Admittance controller

        self.setLayout(self.layout)

        # List to store running SSH processes
        self.driver_processes = []

    def get_selected_robots(self):
        """Returns a list of selected robots."""
        return [name for name, checkbox in self.robots.items() if checkbox.isChecked()]

    def get_selected_urs(self):
        """Returns a list of selected UR prefixes."""
        ur_prefixes = []
        if self.ur10_l.isChecked():
            ur_prefixes.append("UR10_l")
        if self.ur10_r.isChecked():
            ur_prefixes.append("UR10_r")
        return ur_prefixes

    def launch_ros(self, package, launch_file):
        """Launches a ROS launch file with dynamic parameters."""
        selected_robots = self.get_selected_robots()
        robot_names_str = "[" + ",".join(f"'{r}'" for r in selected_robots) + "]"

        command = f"roslaunch {package} {launch_file} robot_names:={robot_names_str}"
        print(f"Executing: {command}")
        subprocess.Popen(command, shell=True)

    def run_compute_object_center(self):
        """Starts compute_object_center.launch with the selected robots as parameters."""
        selected_robots = self.get_selected_robots()

        if not selected_robots:
            print("No robots selected. Skipping launch.")
            return

        # Correct string format for ROS parameter: '["mur620a", "mur620b", "mur620c"]'
        robot_names_str = '"' + str(selected_robots).replace("'", '"') + '"'

        command = f"roslaunch handling compute_object_center.launch robot_names:={robot_names_str}"
        print(f"Executing: {command}")
        subprocess.Popen(command, shell=True)

    def zero_ft_sensors(self):
        """Zeros the selected F/T sensors."""
        selected_robots = self.get_selected_robots()
        selected_urs = self.get_selected_urs()

        if not selected_robots or not selected_urs:
            print("No robots or URs selected. Skipping launch.")
            return

        robot_names_str = '"' + str(selected_robots).replace("'", '"') + '"'
        ur_prefixes_str = '"' + str(selected_urs).replace("'", '"') + '"'

        command = f"roslaunch handling zero_all_FT_sensors.launch robot_names:={robot_names_str} UR_prefixes:={ur_prefixes_str}"
        print(f"Executing: {command}")
        subprocess.Popen(command, shell=True)

    def start_status_update(self):
        """Starts the status update in a separate thread to prevent GUI freezing."""
        threading.Thread(target=self.update_status, daemon=True).start()

    def update_status(self):
        """Checks the status of ROS controllers using the controller_manager service."""
        selected_robots = self.get_selected_robots()
        selected_urs = self.get_selected_urs()
        active_counts = {"wrench": 0, "twist": 0, "arm": 0, "admittance": 0}
        total_count = len(selected_robots) * len(selected_urs)
        
        for robot in selected_robots:
            for ur in selected_urs:
                service_name = f"/{robot}/{ur}/controller_manager/list_controllers"
                try:
                    output = subprocess.check_output(f"rosservice call {service_name}", shell=True).decode()
                    controllers = yaml.safe_load(output).get("controller", [])
                    
                    for controller in controllers:
                        if controller.get("state") == "running":
                            if controller.get("name") == "force_torque_sensor_controller":
                                active_counts["wrench"] += 1
                            if controller.get("name") == "twist_controller":
                                active_counts["twist"] += 1
                            if controller.get("name") == "arm_controller":
                                active_counts["arm"] += 1
                            if controller.get("name") == "admittance_controller":
                                active_counts["admittance"] += 1
                except Exception:
                    pass

        status_text = """
        Wrench Controller: {} of {} active {}
        Twist Controller: {} of {} active {}
        Arm Controller: {} of {} active {}
        Admittance Controller: {} of {} active {}
        """.format(
            active_counts["wrench"], total_count, self.get_status_symbol(active_counts["wrench"], total_count),
            active_counts["twist"], total_count, self.get_status_symbol(active_counts["twist"], total_count),
            active_counts["arm"], total_count, self.get_status_symbol(active_counts["arm"], total_count),
            active_counts["admittance"], total_count, self.get_status_symbol(active_counts["admittance"], total_count),
        )
        
        self.status_label.setText(status_text)
        
        # Highlight Admittance Controller if wrench and twist are fully active
        if active_counts["wrench"] == total_count and active_counts["twist"] == total_count:
            self.status_label.setStyleSheet("background-color: red; color: white; font-weight: bold; padding: 5px;")
        else:
            self.status_label.setStyleSheet("border: 1px solid black; padding: 5px;")

    def get_status_symbol(self, active, total):
        """Returns appropriate status symbol based on active controller count."""
        if active == total:
            return "✅"
        elif active > 0:
            return "⚠️"
        return "❌"


    def turn_on_coop_admittance_controller(self):
        """SSH into each selected robot and start the cooperative admittance controller."""
        selected_robots = self.get_selected_robots()
        selected_urs = self.get_selected_urs()
        set_reference = "true" if self.check_set_reference.isChecked() else "false"

        if not selected_robots or not selected_urs:
            print("No robots or URs selected. Skipping launch.")
            return

        for robot in selected_robots:
            for ur_prefix in selected_urs:
                command = f"ssh -t -t {robot} 'source ~/.bashrc; export ROS_MASTER_URI=http://roscore:11311/; source /opt/ros/noetic/setup.bash; source ~/{self.workspace_name}/devel/setup.bash; roslaunch manipulator_control dezentralized_admittance_controller.launch tf_prefix:={robot} UR_prefix:={ur_prefix} set_reference_at_runtime:={set_reference}; exec bash'"
                print(f"Executing SSH Command: {command}")

                # Open a new terminal and run the SSH command
                subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"])


    def turn_on_wrench_controllers(self):
        """Turns on all wrench controllers for the selected robots."""
        selected_robots = self.get_selected_robots()
        selected_urs = self.get_selected_urs()

        if not selected_robots or not selected_urs:
            print("No robots or URs selected. Skipping launch.")
            return

        robot_names_str = '"' + str(selected_robots).replace("'", '"') + '"'
        ur_prefixes_str = '"' + str(selected_urs).replace("'", '"') + '"'

        command = f"roslaunch handling turn_on_all_wrench_controllers.launch robot_names:={robot_names_str} UR_prefixes:={ur_prefixes_str}"
        print(f"Executing: {command}")
        subprocess.Popen(command, shell=True)

    def turn_on_arm_controllers(self):
        """Turns on all arm controllers for the selected robots."""
        selected_robots = self.get_selected_robots()
        selected_urs = self.get_selected_urs()

        if not selected_robots or not selected_urs:
            print("No robots or URs selected. Skipping launch.")
            return

        robot_names_str = '"' + str(selected_robots).replace("'", '"') + '"'
        ur_prefixes_str = '"' + str(selected_urs).replace("'", '"') + '"'

        command = f"roslaunch handling turn_on_all_arm_controllers.launch robot_names:={robot_names_str} UR_prefixes:={ur_prefixes_str}"
        print(f"Executing: {command}")
        subprocess.Popen(command, shell=True)

    def turn_on_twist_controllers(self):
        """Turns on all twist controllers for the selected robots."""
        selected_robots = self.get_selected_robots()
        selected_urs = self.get_selected_urs()

        if not selected_robots or not selected_urs:
            print("No robots or URs selected. Skipping launch.")
            return

        robot_names_str = '"' + str(selected_robots).replace("'", '"') + '"'
        ur_prefixes_str = '"' + str(selected_urs).replace("'", '"') + '"'

        command = f"roslaunch handling turn_on_all_twist_controllers.launch robot_names:={robot_names_str} UR_prefixes:={ur_prefixes_str}"
        print(f"Executing: {command}")
        subprocess.Popen(command, shell=True)

        
    def enable_all_urs(self):
        """Enables all UR robots for the selected configurations."""
        selected_robots = self.get_selected_robots()
        selected_urs = self.get_selected_urs()

        if not selected_robots or not selected_urs:
            print("No robots or URs selected. Skipping launch.")
            return

        robot_names_str = '"' + str(selected_robots).replace("'", '"') + '"'
        ur_prefixes_str = '"' + str(selected_urs).replace("'", '"') + '"'

        command = f"roslaunch handling enable_all_URs.launch robot_names:={robot_names_str} UR_prefixes:={ur_prefixes_str}"
        print(f"Executing: {command}")
        subprocess.Popen(command, shell=True)

    def update_ur_relative_to_object(self):
        """Updates the relative poses of UR robots to the object."""
        selected_robots = self.get_selected_robots()
        selected_urs = self.get_selected_urs()

        if not selected_robots or not selected_urs:
            print("No robots or URs selected. Skipping launch.")
            return

        robot_names_str = '"' + str(selected_robots).replace("'", '"') + '"'
        ur_prefixes_str = '"' + str(selected_urs).replace("'", '"') + '"'

        command = f"roslaunch handling update_all_relative_poses.launch robot_names:={robot_names_str} UR_prefixes:={ur_prefixes_str}"
        print(f"Executing: {command}")
        subprocess.Popen(command, shell=True)


    def launch_drivers(self):
        """SSH into the selected robots and start the drivers in separate terminals."""
        selected_robots = self.get_selected_robots()

        for robot in selected_robots:
            workspace = self.workspace_name
            command = f"ssh -t -t {robot} 'source ~/.bashrc; export ROS_MASTER_URI=http://roscore:11311/; source /opt/ros/noetic/setup.bash; source ~/{workspace}/devel/setup.bash; roslaunch mur_launch_hardware {robot}.launch; exec bash'"
            print(f"Opening SSH session and launching driver for: {robot}")

            # Open a new terminal with SSH session + driver launch + keep open
            subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"])

    def quit_drivers(self):
        """Terminates all running driver sessions and closes terminals."""
        print("Stopping all driver sessions...")
        try:
            subprocess.Popen("pkill -f 'ssh -t -t'", shell=True)
            subprocess.Popen("pkill -f 'gnome-terminal'", shell=True)
        except Exception as e:
            print(f"Error stopping processes: {e}")


    def move_to_initial_pose(self, UR_prefix):
        """Moves the selected robots to the initial pose with the correct namespace and move_group_name."""
        selected_robots = self.get_selected_robots()

        # Set move_group_name based on UR_prefix
        move_group_name = "UR_arm_l" if UR_prefix == "UR10_l" else "UR_arm_r"

        for robot in selected_robots:
            # Set home_position based on robot name
            if robot in ["mur620a", "mur620b"]:
                home_position = "handling_position_wide"
            else:  # mur620c, mur620d
                home_position = "handling_position_wide_lift"

            # ROS launch command with namespace
            command = f"ROS_NAMESPACE={robot} roslaunch ur_utilities move_UR_to_home_pose.launch tf_prefix:={robot} UR_prefix:={UR_prefix} home_position:={home_position} move_group_name:={move_group_name}"
            print(f"Executing: {command}")
            subprocess.Popen(command, shell=True)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ROSGui()
    window.show()
    sys.exit(app.exec_())
