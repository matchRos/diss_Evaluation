import sys
import subprocess
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QCheckBox

class ROSGui(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Multi-Robot Demo")
        self.setGeometry(100, 100, 350, 350)

        self.layout = QVBoxLayout()

        # Checkboxes für Roboter
        self.robots = {
            "mur620a": QCheckBox("mur620a"),
            "mur620b": QCheckBox("mur620b"),
            "mur620c": QCheckBox("mur620c"),
            "mur620d": QCheckBox("mur620d"),
        }

        for checkbox in self.robots.values():
            self.layout.addWidget(checkbox)

        # Buttons für das Starten der ROS-Launch-Files
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

        self.layout.addWidget(self.btn_virtual_leader)
        self.layout.addWidget(self.btn_virtual_object)
        self.layout.addWidget(self.btn_compute_center)
        self.layout.addWidget(self.btn_launch_drivers)
        self.layout.addWidget(self.btn_quit_drivers)

        self.setLayout(self.layout)

        # Liste zum Speichern der laufenden SSH-Prozesse
        self.driver_processes = []

    def get_selected_robots(self):
        """Liest die aktivierten Roboter aus und gibt sie als Liste zurück."""
        return [name for name, checkbox in self.robots.items() if checkbox.isChecked()]

    def launch_ros(self, package, launch_file):
        """Startet ein ROS-Launch-File mit dynamischen Parametern."""
        selected_robots = self.get_selected_robots()
        robot_names_str = "[" + ",".join(f"'{r}'" for r in selected_robots) + "]"

        command = f"roslaunch {package} {launch_file} robot_names:={robot_names_str}"
        print(f"Executing: {command}")
        subprocess.Popen(command, shell=True)

    def run_compute_object_center(self):
        """Startet compute_object_center.py mit den gewählten Robotern als Parameter."""
        selected_robots = self.get_selected_robots()
        robot_names_str = ",".join(selected_robots)

        command = f"rosrun handling compute_object_center.py _robot_names:={robot_names_str}"
        print(f"Executing: {command}")
        subprocess.Popen(command, shell=True)

    def launch_drivers(self):
        """SSH zu den ausgewählten Robotern und starte die Treiber in separaten Terminals."""
        selected_robots = self.get_selected_robots()

        for robot in selected_robots:
            command = f"ssh {robot} 'source /opt/ros/noetic/setup.bash && source ~/catkin_ws_recker/devel/setup.bash && roslaunch mur_launch_hardware {robot}.launch; exec bash'"

            print(f"Executing SSH Command: {command}")

            # Terminal starten und SSH-Session offen halten
            process = subprocess.Popen(
                ["gnome-terminal", "--", "bash", "-c", command],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            # Prozess speichern, um ihn später beenden zu können
            self.driver_processes.append(process)


    def quit_drivers(self):
        """Beendet alle laufenden SSH-Sitzungen."""
        print("Stopping all driver sessions...")

        for process in self.driver_processes:
            try:
                process.terminate()  # Versucht den Prozess sauber zu beenden
            except Exception as e:
                print(f"Error stopping process: {e}")

        self.driver_processes.clear()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ROSGui()
    window.show()
    sys.exit(app.exec_())
