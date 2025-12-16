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
from my_robot_interfaces.srv import MoveToXYZ
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient

#PySide imports
from PySide6 import QtCore
from PySide6.QtWidgets import QApplication, QSlider, QLabel, QComboBox, QLineEdit, QPushButton, QTableView, QHeaderView
from PySide6.QtUiTools import QUiLoader
from PySide6.QtCore import QFile
from PySide6.QtGui import QStandardItemModel, QStandardItem


class JazzyGuiNode(Node):
  
    def __init__(self, node_name: str = 'jazzy_gui_node'):
        super().__init__(node_name)

        #Create a publisher for the angle values of the slider 
        # self.slider_publisher = self.create_publisher(JointState, 'joint_states', 10) #################
        # self.msg = JointState()
        # self.msg.name = ['gripper', 'joint_1', 'joint_2', 'joint_3']
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        #Create a publisher for the Controll-Selection
        self.controll_state_publisher = self.create_publisher(Bool, 'controll_state', 10)
        self.gui_controll_enabled = Bool()
        self.gui_controll_enabled.data = True

        #Create a publisher for the robot mode
        self.robot_mode_publisher = self.create_publisher(String, 'robot_mode', 10)
        self.robot_mode = String()
        self.robot_mode.data = "Simulation Mode"

        #Create the service client to use the MovetoXYZ service
        self.cli = self.create_client(MoveToXYZ, 'move_to_xyz')
        
        # Wait till the service is available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Warte auf Service /move_to_xyz...')
        # Prepare the request
        self.req = MoveToXYZ.Request()
    
        self.get_logger().info('JazzyGuiNode initialisiert')

class UiWindow:
    
    def __init__(self, ui_path: str, node: JazzyGuiNode):
        self.node = node
        self.ui_path = ui_path

        #Create a subscriber for to receive the forward kinematics information
        self.fk_subscriber = self.node.create_subscription(PoseStamped, 'fk_info', self.update_line_edits, 10)

        #Create a subscriber to update the slider values when in pot controll mode
        self.pot_subscriber = self.node.create_subscription(JointState, 'joint_states', self.sync_sliders_to_movement, 10)

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

        self.sequence_counter = 1
        self.is_sequence_running = False
        self.sync_sliders = False
        self.current_seq_index = 0

        #initiate all the sliders 
        self.init_sliders()

        #initialise the Dropdown-Menu to choose the robot Mode
        self.init_mode_dropdown()
        
        #initialise the Dropdown-Menu to choose the controll type
        self.init_controll_dropdown() 

        #initiate the QLineEdits
        self.init_line_edits()

        #initiate all the push buttons
        self.init_pushbuttons()

        #initiate the sequential table 
        self.init_sequential_table()

        #initiate and start a timer to run ros2 callbacks
        self.ros_spin_time = 100 #ms
        self.ros_spin_timer = QtCore.QTimer()
        self.ros_spin_timer.timeout.connect(self.spin_ros_node)
        self.ros_spin_timer.start(self.ros_spin_time)

        #publish the initial home pose of the robot
        # #define the values of the joints and the gripper in radiants
        # self.joint1_home = 0
        # self.joint2_home = -pi/16
        # self.joint3_home = pi/2
        # self.gripper_home = pi/4
        # self.home_pos = [self.joint1_home, self.joint2_home, self.joint3_home, self.gripper_home] 

        # #publish the values
        # self.node.msg.position = self.home_pos
        # self.node.msg.header.stamp = self.node.get_clock().now().to_msg()
        # self.node.slider_publisher.publish(self.node.msg)

        # # Convert radians → 0–180°
        # deg_1, deg_2, deg_3, deg_gripper = self.rad_vec_to_deg_0_180(self.home_pos)
        
        # # Set the initial slider values according to the home position
        # self.slider_joint_1.setValue(deg_1)
        # self.slider_joint_2.setValue(deg_2)
        # self.slider_joint_3.setValue(deg_3)
        # self.slider_gripper.setValue(deg_gripper)

        
    def init_sliders(self):

        #get the sliders from the UI
        self.slider_joint_1 = self.window.findChild(QSlider, "slider_joint1")
        self.slider_joint_2 = self.window.findChild(QSlider, "slider_joint2")
        self.slider_joint_3 = self.window.findChild(QSlider, "slider_joint3")
        self.slider_gripper = self.window.findChild(QSlider, "gripper_slider")

        self.slider_arr = [self.slider_joint_1, self.slider_joint_2, self.slider_joint_3]

        #set the value step and range for each slider 
        for slider in self.slider_arr:
            slider.setMinimum(0)
            slider.setMaximum(180)
            slider.setValue(90)
            slider.setSingleStep(1)
            slider.valueChanged.connect(self.joint_slider_changed)

        self.slider_gripper.setMinimum(90)
        self.slider_gripper.setMaximum(180)
        self.slider_gripper.setValue(90)
        self.slider_gripper.setSingleStep(1)
        self.slider_gripper.valueChanged.connect(self.gripper_slider_changed)

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

        self.current_pos_x.setText("--")
        self.current_pos_y.setText("--")
        self.current_pos_z.setText("--")

        self.desired_pos_x = self.window.findChild(QLineEdit, "dessired_pos_x")
        self.desired_pos_y = self.window.findChild(QLineEdit, "dessired_pos_y")
        self.desired_pos_z = self.window.findChild(QLineEdit, "dessired_pos_z")

        self.desired_pos_x.setText("--")
        self.desired_pos_y.setText("--")
        self.desired_pos_z.setText("--")

    def init_pushbuttons(self):

        self.pushButton_send_xyz = self.window.findChild(QPushButton, "pushButton_send_xyz")
        self.pushButton_send_xyz.clicked.connect(self.send_desired_position)

        self.pushButton_save_current_pos = self.window.findChild(QPushButton, "pushButton_save_current_pos")
        self.pushButton_save_current_pos.clicked.connect(self.save_current_position)

        self.pushButton_save_grip = self.window.findChild(QPushButton, "pushButton_save_grip")
        self.pushButton_save_grip.clicked.connect(self.save_gripper_state)

        self.pushButton_delete_pos = self.window.findChild(QPushButton, "pushButton_delete_pos")
        self.pushButton_delete_pos.clicked.connect(self.delete_selected_position)

        self.pushButton_send_seq = self.window.findChild(QPushButton, "pushButton_send_seq")
        self.pushButton_send_seq.clicked.connect(self.send_position_sequence)

    def init_sequential_table(self):

        self.sequential_table = self.window.findChild(QTableView, "sequential_table")

        # Model
        self.sequence_model = QStandardItemModel(0, 4)
        self.sequence_model.setHorizontalHeaderLabels(
        ["Type", "X", "Y", "Z"]
        )
        self.sequential_table.setModel(self.sequence_model)

        # Header / Spalten
        header = self.sequential_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Fixed)
        header.setStretchLastSection(False)

        self.sequential_table.setColumnWidth(0, 104)  # Name
        self.sequential_table.setColumnWidth(1, 62)  # X
        self.sequential_table.setColumnWidth(2, 62)  # Y
        self.sequential_table.setColumnWidth(3, 62)  # Z

        # Optik
        self.sequential_table.verticalHeader().setVisible(False)
        self.sequential_table.setShowGrid(True)
        self.sequential_table.setAlternatingRowColors(True)
        self.sequential_table.setSelectionBehavior(QTableView.SelectRows)
        self.sequential_table.setSelectionMode(QTableView.SingleSelection)

        self.sequence_counter = 1

    def update_control_state(self, text):
        if text == "GUI-Control":
            self.node.gui_controll_enabled.data = True
            self.sync_sliders = False
            self.node.get_logger().info("GUI Controll Enabled")
        else:
            self.node.gui_controll_enabled.data = False
            self.sync_sliders = True
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

        self.current_pos_x.setText(f"{x_mm}")
        self.current_pos_y.setText(f"{y_mm}")
        self.current_pos_z.setText(f"{z_mm}")
    
    def sync_sliders_to_movement(self, msg):
        if not self.sync_sliders:
            return

        num_positions = len(msg.position)
        if num_positions != 4:
            return

        # Convert radians → 0–180°
        deg_1, deg_2, deg_3, deg_4 = self.rad_vec_to_deg_0_180(msg.position)

        # Block Qt signals to avoid recursive callbacks
        self.slider_joint_1.blockSignals(True)
        self.slider_joint_2.blockSignals(True)
        self.slider_joint_3.blockSignals(True)
        self.slider_gripper.blockSignals(True)

        self.slider_gripper.setValue(deg_1)
        self.slider_joint_1.setValue(deg_2)
        self.slider_joint_2.setValue(deg_3)
        self.slider_joint_3.setValue(deg_4)

        self.slider_joint_1.blockSignals(False)
        self.slider_joint_2.blockSignals(False)
        self.slider_joint_3.blockSignals(False)
        self.slider_gripper.blockSignals(False)

        # Update labels manually
        self.slider_label_1.setText(f"Joint Angle 1: {deg_2} degrees")
        self.slider_label_2.setText(f"Joint Angle 2: {deg_3} degrees")
        self.slider_label_3.setText(f"Joint Angle 3: {deg_4} degrees")

    def set_sync_sliders_flag(self, flag):
        self.sync_sliders = flag

    def show(self):
        self.window.show()  

    def rad_vec_to_deg_0_180(self, rad_list):
        return [int(round((r * 180 / pi) + 90)) for r in rad_list] 
    
    def send_position_sequence(self):
        if self.sequence_model.rowCount() == 0:
            return
        
        self.is_sequence_running = True
        self.current_seq_index = 0
    
        # Starte den ersten Punkt
        self.send_desired_position(self.current_seq_index, sequence=True)

    def send_desired_position(self, pos_number=0, sequence=False):
        self.set_sync_sliders_flag(True)
        self.node.get_logger().info(f"sync_sliders = {self.sync_sliders}")

        try:
            if sequence:
                # 1. Typ der Zeile auslesen
                cmd_type = self.sequence_model.item(pos_number, 0).text()  

                if cmd_type == "MOVE":
                    
                    name = self.sequence_model.item(pos_number, 0).text()
                    x_val = float(self.sequence_model.item(pos_number, 1).text()) / 100.0
                    y_val = float(self.sequence_model.item(pos_number, 2).text()) / 100.0
                    z_val = float(self.sequence_model.item(pos_number, 3).text()) / 100.0

                    self.node.get_logger().info(f"Sending position request for {name}: X={x_val}, Y={y_val}, Z={z_val}")

                elif cmd_type == "GRIPPER":
                    val_str = self.sequence_model.item(pos_number, 1).text()
                    closure_percent = float(val_str)
                    position_rad = (closure_percent / 100.0) * (pi / 2.0)

                    goal = GripperCommand.Goal()
                    goal.command.position = position_rad

                    self.node.get_logger().info(f"Sending Gripper Command: {closure_percent:.2f} % closed")
                    send_goal_future = self.node.gripper_client.send_goal_async(goal)

                    send_goal_future.add_done_callback(self.gripper_get_result_callback)
                    return

            else:# Read the text of the input boxes and change the unit to meters
                x_val = float(self.desired_pos_x.text()) / 100.0
                y_val = float(self.desired_pos_y.text()) / 100.0
                z_val = float(self.desired_pos_z.text()) / 100.0

                self.node.get_logger().info(f"Sende Service Anfrage: X={x_val}, Y={y_val}, Z={z_val}")

            # Fill the Request
            self.node.req.x = x_val
            self.node.req.y = y_val
            self.node.req.z = z_val

            # Send the request asynchronosly so the GUI doesnt freeze
            future = self.node.cli.call_async(self.node.req)
            
            # Define the function that is called when the service is done 
            future.add_done_callback(self.service_response_callback)

        except ValueError:
            self.node.get_logger().error("Please make sure the inputs are numbers!")

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info(f"Erfolg: {response.message}")
                QtCore.QTimer.singleShot(2000, lambda: self.set_sync_sliders_flag(False))
                self.node.get_logger().info(f"sync_sliders = {self.sync_sliders}")
            else:
                self.node.get_logger().error(f"Fehler: {response.message}")
                QtCore.QTimer.singleShot(2000, lambda: self.set_sync_sliders_flag(False))
                self.node.get_logger().info(f"sync_sliders = {self.sync_sliders}")
        except Exception as e:
            self.node.get_logger().error(f"Service Call fehlgeschlagen: {e}")
            QtCore.QTimer.singleShot(2000, lambda: self.set_sync_sliders_flag(False))
            self.node.get_logger().info(f"sync_sliders = {self.sync_sliders}")
        
        if self.is_sequence_running:
            self.current_seq_index += 1 
        
            # Prüfen, ob noch Zeilen übrig sind
            if self.current_seq_index < self.sequence_model.rowCount():
                self.node.get_logger().info(f"Starting next position: {self.current_seq_index}")
                # Nächsten Punkt senden
                self.send_desired_position(self.current_seq_index, sequence=True)
            else:
                self.node.get_logger().info("Sequence finished.")
                QtCore.QTimer.singleShot(2000, lambda: self.set_sync_sliders_flag(False))
                
                QtCore.QTimer.singleShot(2000, lambda: self.set_sync_sliders_flag(False))
                self.node.get_logger().info(f"sync_sliders = {self.sync_sliders}")

    def gripper_get_result_callback(self, future):
        # Greifer ist fertig!
        self.node.get_logger().info('Greifer-Aktion abgeschlossen.')

        # Das ist die gleiche Logik wie in service_response_callback
        if self.is_sequence_running:
            self.current_seq_index += 1
            if self.current_seq_index < self.sequence_model.rowCount():
                self.send_desired_position(self.current_seq_index, sequence=True)
                
            else:
                self.node.get_logger().info("Sequenz vollständig beendet.")
                self.is_sequence_running = False

    def save_current_position(self):
        x = self.current_pos_x.text()
        y = self.current_pos_y.text()
        z = self.current_pos_z.text()

        if "--" in (x, y, z):
            self.node.get_logger().error("Current position not available")
            return

        row_items = [
            QStandardItem(f"MOVE"),
            QStandardItem(x),
            QStandardItem(y),
            QStandardItem(z),
        ]

        self.sequence_model.appendRow(row_items)
        self.sequence_counter += 1
    
    def save_gripper_state(self):
        # Aktuellen Wert vom Slider holen
        closure_percent = (self.slider_gripper.value() - 90.0) / 90.0 * 100.0
        closure_str = str(closure_percent)

        row_items = [
            QStandardItem("GRIPPER"), # Typ
            QStandardItem(str(closure_str)),  # Val 1 (Winkel)
            QStandardItem("% Closed"),       # Val 2 (leer)
            QStandardItem("-")        # Val 3 (leer)
        ]
        self.sequence_model.appendRow(row_items)
        self.sequence_counter += 1

    def delete_selected_position(self):
        selection_model = self.sequential_table.selectionModel()
        if not selection_model.hasSelection():
            self.node.get_logger().warn("Keine Zeile ausgewaehlt")
            return

        row = selection_model.selectedRows()[0].row()
        self.sequence_model.removeRow(row)

    @QtCore.Slot()
    def spin_ros_node(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    @QtCore.Slot(int)
    def joint_slider_changed(self, value): 

        # self.node.get_logger().info(f"Published slider value: {self.node.msg.position}")

        # value_1 = float(self.slider_gripper.value())
        value_2 = float(self.slider_joint_1.value())
        value_3 = float(self.slider_joint_2.value())
        value_4 = float(self.slider_joint_3.value())
        

        self.slider_label_1.setText(f"Joint Angle 1: {value_2} degrees")
        self.slider_label_2.setText(f"Joint Angle 2: {value_3} degrees")
        self.slider_label_3.setText(f"Joint Angle 3: {value_4} degrees")

        # Convert 0-180 -> -pi/2 ... pi/2
        # rad_1 = (value_1 - 90) * pi / 180
        rad_2 = (value_2 - 90) * pi / 180
        rad_3 = (value_3 - 90) * pi / 180
        rad_4 = (value_4 - 90) * pi / 180

        # if self.node.gui_controll_enabled.data:

        #     self.node.msg.position = [rad_1, rad_2, rad_3, rad_4]
        #     self.node.msg.header.stamp = self.node.get_clock().now().to_msg()
        #     self.node.slider_publisher.publish(self.node.msg)

        # self.node.get_logger().info(f"Published slider value: {self.node.msg.position}")
        if self.node.gui_controll_enabled.data:
            traj = FollowJointTrajectory.Goal()
            traj.trajectory.joint_names = ['joint_1','joint_2','joint_3']

            point = JointTrajectoryPoint()
            point.positions = [rad_2, rad_3, rad_4]
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 300_000_000  # 300 ms

            traj.trajectory.points.append(point)
            self.node.trajectory_client.send_goal_async(traj)
    
    def gripper_slider_changed(self, value):
        # 90–180 → 0.0–1.0
        position = (value - 90) * pi / 180.0

        goal = GripperCommand.Goal()
        goal.command.position = position

        self.node.gripper_client.send_goal_async(goal)
    

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