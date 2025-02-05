import sys
import subprocess
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QCheckBox, QLabel

class ROSGui(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Multi-Robot Demo")
        self.setGeometry(100, 100, 400, 450)
        self.workspace_name = "catkin_ws_recker"
        self.layout = QVBoxLayout()

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

        self.btn_zero_ft_sensors = QPushButton("Zero F/T Sensors")
        self.btn_zero_ft_sensors.clicked.connect(self.zero_ft_sensors)

        # Buttons for Initial Pose (left & right)
        move_pose_layout = QHBoxLayout()
        self.btn_move_left = QPushButton("Move to Initial Pose Left")
        self.btn_move_left.clicked.connect(lambda: self.move_to_initial_pose("UR10_l"))
        self.btn_move_right = QPushButton("Move to Initial Pose Right")
        self.btn_move_right.clicked.connect(lambda: self.move_to_initial_pose("UR10_r"))

        move_pose_layout.addWidget(self.btn_move_left)
        move_pose_layout.addWidget(self.btn_move_right)

        # Add elements to the main layout
        self.layout.addLayout(robot_ur_layout)  # Robot & UR checkboxes
        self.layout.addWidget(self.btn_virtual_leader)
        self.layout.addWidget(self.btn_virtual_object)
        self.layout.addWidget(self.btn_compute_center)
        self.layout.addWidget(self.btn_launch_drivers)
        self.layout.addWidget(self.btn_quit_drivers)
        self.layout.addWidget(self.btn_zero_ft_sensors)
        self.layout.addLayout(move_pose_layout)  # Move buttons

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
        """Stops all running SSH sessions."""
        print("Stopping all driver sessions...")
        for process in self.driver_processes:
            try:
                process.terminate()
            except Exception as e:
                print(f"Error stopping process: {e}")
        self.driver_processes.clear()

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
