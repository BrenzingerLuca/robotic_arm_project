#necessary to run the GUI and find the .ui file
import sys
from pathlib import Path

#ROS2 specific imports
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

#PySide imports
from PySide6 import QtCore, QtWidgets, QtGui
from PySide6.QtWidgets import QApplication, QSlider, QLabel, QComboBox
from PySide6.QtUiTools import QUiLoader
from PySide6.QtCore import QFile


class JazzyGuiNode(Node):
  
    def __init__(self, node_name: str = 'jazzy_gui_node'):
        super().__init__(node_name)

        #Create a publisher for the angle values of the slider 
        self.slider_publisher = self.create_publisher(JointState, 'slider_messages', 10)
        self.msg = JointState()
        self.msg.name = ['joint1', 'joint2', 'joint3']

        #Create a publisher for the Controll-Selection
        self.controll_state_publisher = self.create_publisher(Bool, 'controll_state', 10)
        self.gui_controll_enabled = Bool()
        self.gui_controll_enabled.data = True
    
        self.get_logger().info('JazzyGuiNode initialisiert')



class UiWindow:
    
    def __init__(self, ui_path: str, node: JazzyGuiNode):
        self.node = node
        self.ui_path = ui_path

        #loading the .ui file generated with the QtDesigner
        loader = QUiLoader()
        ui_file = QFile(self.ui_path)
        if not ui_file.open(QFile.ReadOnly):
            raise RuntimeError(f"Kann UI-Datei nicht öffnen: {self.ui_path}")
        try:
            self.window = loader.load(ui_file)
        finally:
            ui_file.close()

        if self.window is None:
            raise RuntimeError(f"UI-Laden fehlgeschlagen: {self.ui_path}")

        #initiate all the sliders 
        self.init_sliders()

        #initialise the Dropdown-Menu to choose the controll type
        self.init_controll_dropdown() 

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

    def show(self):
        self.window.show()

    def update_control_state(self, text):
        if text == "GUI-Control":
            self.node.gui_controll_enabled.data = True
            self.node.get_logger().info("GUI Controll Enabled")
        else:
            self.node.gui_controll_enabled.data = False
            self.node.get_logger().info("Potentiometer Controll Enabled")
        
        self.node.controll_state_publisher.publish(self.node.gui_controll_enabled)
        
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

        self.node.msg.position = [value_1, value_2, value_3]
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

    try:
        return_code = app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return return_code

if __name__ == '__main__':
    raise SystemExit(main())