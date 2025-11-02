import sys
import random
from PySide6 import QtCore, QtWidgets, QtGui

import rclpy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import JointState

class MyWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.node = rclpy.create_node("Robo_GUI")
        self.button_publisher = self.node.create_publisher(String, 'gui_messages', 10)
        self.slider_publisher = self.node.create_publisher(JointState, 'slider_messages', 10)

        self.msg = JointState()
        self.msg.name = ['joint1', 'joint2', 'joint3']

        self.hello = ["Hallo Welt", "Hei maailma", "Hola Mundo", "Привет мир"]
        self.text = QtWidgets.QLabel("Hello World",alignment=QtCore.Qt.AlignCenter)
        
        #create a Qt timer to spin the ros node to interact with topics
        self.ros_updater = QtCore.QTimer(self)
        self.ros_updater.timeout.connect(self.spin_ros_node)
        self.ros_updater.start(100)

        #add a button to publish values
        self.button = QtWidgets.QPushButton("Change Language")
        self.button.clicked.connect(self.button_clicked)

        #add a slider to control the servo angle
        self.slider_joint_1 = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_joint_2 = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_joint_3 = QtWidgets.QSlider(QtCore.Qt.Horizontal)

        self.slider_arr = [self.slider_joint_1, self.slider_joint_2, self.slider_joint_3]
        
        for slider in self.slider_arr:
            slider.setMinimum(-90.0)
            slider.setMaximum(90.0)
            slider.setValue(0)
            slider.setSingleStep(1.0)
            slider.valueChanged.connect(self.slider_value_changed)

        self.slider_label_1 = QtWidgets.QLabel(f"Angle of base_joint {self.slider_joint_1.value()} degrees", alignment=QtCore.Qt.AlignCenter) 
        self.slider_label_2 = QtWidgets.QLabel(f"Angle of joint 2 {self.slider_joint_2.value()} degrees", alignment=QtCore.Qt.AlignCenter) 
        self.slider_label_3 = QtWidgets.QLabel(f"Angle of joint 3 {self.slider_joint_3.value()} degrees", alignment=QtCore.Qt.AlignCenter) 

        #add every widget to the main window
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.addWidget(self.text)
        self.layout.addWidget(self.button)
        #slider to controll joint 1
        self.layout.addWidget(self.slider_joint_1)
        self.layout.addWidget(self.slider_label_1)
        #slider to controll joint 2
        self.layout.addWidget(self.slider_joint_2)
        self.layout.addWidget(self.slider_label_2)
        #slider to controll joint 3
        self.layout.addWidget(self.slider_joint_3)
        self.layout.addWidget(self.slider_label_3)

    @QtCore.Slot()
    def button_clicked(self):
        selected_message = random.choice(self.hello)
        self.text.setText(selected_message)
        msg = String()
        msg.data = selected_message
        self.button_publisher.publish(msg)
        self.node.get_logger().info(f"Published: '{msg.data}'") 
    
    @QtCore.Slot()
    def spin_ros_node(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    @QtCore.Slot(int)
    def slider_value_changed(self, value): 

        value_1 = float(self.slider_joint_1.value())
        value_2 = float(self.slider_joint_2.value())
        value_3 = float(self.slider_joint_3.value())

        self.slider_label_1.setText(f"Angle of base_joint: {value_1} degrees")
        self.slider_label_2.setText(f"Angle of base_joint: {value_2} degrees")
        self.slider_label_3.setText(f"Angle of base_joint: {value_3} degrees")

        self.msg.position = [value_1, value_2, value_3]
        self.slider_publisher.publish(self.msg)

        self.node.get_logger().info(f"Published slider value: {self.msg.position}")


def main(args=None):
    rclpy.init()

    app = QtWidgets.QApplication([])

    widget = MyWidget()
    widget.resize(1000, 800)
    widget.show()

    app.aboutToQuit.connect(rclpy.shutdown) 
    sys.exit(app.exec())

if __name__ == "__main__":
    main()