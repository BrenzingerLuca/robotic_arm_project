#Necessary to run the GUI and find the .ui file
import sys
import signal
from pathlib import Path

#To be able to use Pi
from math import pi 

#ROS2 specific imports
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

#PySide imports
from PySide6 import QtCore, QtWidgets, QtGui
from PySide6.QtWidgets import QApplication, QSlider, QLabel, QComboBox, QLineEdit
from PySide6.QtUiTools import QUiLoader
from PySide6.QtCore import QFile


class JazzyGuiNode(Node):
  
    def __init__(self, node_name: str = 'jazzy_gui_node'):
        super().__init__(node_name)

        #Create a publisher for the angle values of the slider 
        self.slider_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.msg = JointState()
        self.msg.name = ['joint_1', 'joint_2', 'joint_3']

        #Create a publisher for the Controll-Selection
        self.controll_state_publisher = self.create_publisher(Bool, 'controll_state', 10)
        self.gui_controll_enabled = Bool()
        self.gui_controll_enabled.data = True

        #Create a publisher for the robot mode
        self.robot_mode_publisher = self.create_publisher(String, 'robot_mode', 10)
        self.robot_mode = String()
        self.robot_mode.data = "Simulation Mode"
    
        self.get_logger().info('JazzyGuiNode initialisiert')

class UiWindow:
    
    def __init__(self, ui_path: str, node: JazzyGuiNode):
        self.node = node
        self.ui_path = ui_path

        #Create a subscriber for to receive the forward kinematics information
        self.fk_subscriber = self.node.create_subscription(PoseStamped, 'fk_info', self.update_line_edits, 10)

        #loading the .ui file generated with the QtDesigner
        loader = QUiLoader()
        ui_file = QFile(self.ui_path)
        if not ui_file.open(QFile.ReadOnly):
            raise RuntimeError(f"Can't open UI-File: {self.ui_path}")
        try:
            self.window = loader.load(ui_file)
        finally:
            ui_file.close()

        if self.window is None:
            raise RuntimeError(f"Loading the UI has failed: {self.ui_path}")

        #initiate all the sliders 
        self.init_sliders()

        #initialise the Dropdown-Menu to choose the robot Mode
        self.init_mode_dropdown()
        
        #initialise the Dropdown-Menu to choose the controll type
        self.init_controll_dropdown() 

        #initiate the QLineEdits
        self.init_line_edits()

        #initiate and start a timer to run ros2 callbacks
        self.ros_spin_time = 100 #ms
        self.ros_spin_timer = QtCore.QTimer()
        self.ros_spin_timer.timeout.connect(self.spin_ros_node)
        self.ros_spin_timer.start(self.ros_spin_time)

        
    def init_sliders(self):

        #get the sliders from the UI
        self.slider_joint_1 = self.window.findChild(QSlider, "slider_joint1")
        self.slider_joint_2 = self.window.findChild(QSlider, "slider_joint2")
        self.slider_joint_3 = self.window.findChild(QSlider, "slider_joint3")

        self.slider_arr = [self.slider_joint_1, self.slider_joint_2, self.slider_joint_3]

        #set the value step and range for each slider 
        for slider in self.slider_arr:
            slider.setMinimum(0)
            slider.setMaximum(180)
            slider.setValue(90)
            slider.setSingleStep(1)
            slider.valueChanged.connect(self.slider_value_changed)

        self.slider_label_1 = self.window.findChild(QLabel,"label_slider_joint1")
        self.slider_label_2 = self.window.findChild(QLabel,"label_slider_joint2")
        self.slider_label_3 = self.window.findChild(QLabel,"label_slider_joint3")

        self.node.get_logger().info(f"Initiated and connected all the sliders")

    def init_controll_dropdown(self):

        self.controll_dropdown = self.window.findChild(QComboBox, "controll_state_box")
        self.controll_dropdown.setCurrentIndex(1)
        self.controll_dropdown.currentTextChanged.connect(self.update_control_state)
    
    def init_mode_dropdown(self):

        self.mode_dropdown = self.window.findChild(QComboBox, "mode_dropdown")
        self.mode_dropdown.setCurrentIndex(1)
        self.mode_dropdown.currentTextChanged.connect(self.update_robot_mode)

    def init_line_edits(self):

        self.current_pos_x = self.window.findChild(QLineEdit, "current_pos_x")
        self.current_pos_y = self.window.findChild(QLineEdit, "current_pos_y")
        self.current_pos_z = self.window.findChild(QLineEdit, "current_pos_z")

        self.current_pos_x.setText("X: 0.0")
        self.current_pos_y.setText("Y: 0.0")
        self.current_pos_z.setText("Z: 0.0")

    def update_control_state(self, text):
        if text == "GUI-Control":
            self.node.gui_controll_enabled.data = True
            self.node.get_logger().info("GUI Controll Enabled")
        else:
            self.node.gui_controll_enabled.data = False
            self.node.get_logger().info("Potentiometer Controll Enabled")
        
        self.node.controll_state_publisher.publish(self.node.gui_controll_enabled)
    
    def update_robot_mode(self, text):
        if text == "Hardware Mode":
            self.node.robot_mode.data = "Hardware Mode"
            self.node.get_logger().info("Changed Robot Mode to: Hardware Mode")
        elif text == "Simulation Mode":
            self.node.robot_mode.data = "Simulation Mode"
            self.node.get_logger().info("Changed Robot Mode to: Simulation Mode")
        
        self.node.robot_mode_publisher.publish(self.node.robot_mode)

    def update_line_edits(self, msg):

        x_mm = int(round(msg.pose.position.x * 100))
        y_mm = int(round(msg.pose.position.y * 100))
        z_mm = int(round(msg.pose.position.z * 100))

        self.current_pos_x.setText(f"X: {x_mm} mm")
        self.current_pos_y.setText(f"Y: {y_mm} mm")
        self.current_pos_z.setText(f"Z: {z_mm} mm")

    def show(self):
        self.window.show()   

    @QtCore.Slot()
    def spin_ros_node(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    @QtCore.Slot(int)
    def slider_value_changed(self, value): 

        self.node.get_logger().info(f"Published slider value: {self.node.msg.position}")

        value_1 = float(self.slider_joint_1.value())
        value_2 = float(self.slider_joint_2.value())
        value_3 = float(self.slider_joint_3.value())

        self.slider_label_1.setText(f"Joint Angle 1: {value_1} degrees")
        self.slider_label_2.setText(f"Joint Angle 2: {value_2} degrees")
        self.slider_label_3.setText(f"Joint Angle 3: {value_3} degrees")

        # Convert 0-180 -> -pi/2 ... pi/2
        rad_1 = (value_1 - 90) * pi / 180
        rad_2 = (value_2 - 90) * pi / 180
        rad_3 = (value_3 - 90) * pi / 180

        if self.node.gui_controll_enabled.data:

            self.node.msg.position = [rad_1, rad_2, rad_3]
            self.node.msg.header.stamp = self.node.get_clock().now().to_msg()
            self.node.slider_publisher.publish(self.node.msg)

        self.node.get_logger().info(f"Published slider value: {self.node.msg.position}")

    

def main(argv=None):
    argv = argv or sys.argv

    # rclpy init
    rclpy.init(args=argv)
    node = JazzyGuiNode()

    # finde UI-Datei im package share
    pkg_share = get_package_share_directory('robot_gui') 
    ui_path = Path(pkg_share) / 'ui' / 'robot_gui.ui'      

    #exception if the .ui file cant be found
    if not ui_path.exists():
        node.get_logger().error(f"UI-Datei nicht gefunden: {ui_path}")
        rclpy.shutdown()
        return 1


    app = QApplication(argv)
    ui = UiWindow(str(ui_path), node)
    ui.show()

    #Signal-Handler to close the window on Strg+C
    def signal_handler(sig, frame):
        node.get_logger().info("SIGINT empfangen, schließe GUI...")
        ui.window.close()          # Fenster schließen
        node.destroy_node()        # ROS2 Node sauber zerstören
        rclpy.shutdown()           # ROS2 sauber herunterfahren
        app.quit()                 # Qt Event Loop beenden

    signal.signal(signal.SIGINT, signal_handler)

    try:
        return_code = app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return return_code

if __name__ == '__main__':
    raise SystemExit(main())