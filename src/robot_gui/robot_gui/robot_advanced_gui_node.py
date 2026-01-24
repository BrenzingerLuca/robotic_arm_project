#Necessary to run the GUI and find the .ui file
import sys
import signal
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

# Standard Import
from math import pi 

# ROS2 specific imports
import rclpy
from rclpy.node import Node

# ROS 2 message and service types
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray

# Custom service interface
from my_robot_interfaces.srv import MoveToXYZ

# Qt / PySide6 Imports
from PySide6 import QtCore
from PySide6.QtWidgets import QApplication, QSlider, QLabel, QComboBox, QLineEdit, QPushButton, QTableView, QHeaderView
from PySide6.QtUiTools import QUiLoader
from PySide6.QtCore import QFile
from PySide6.QtGui import QStandardItemModel, QStandardItem


class JazzyGuiNode(Node):

    """
    ROS 2 node that provides all communication interfaces
    for the GUI:
    - Action clients for arm and gripper
    - Publishers for control state and robot mode
    - Service client for MoveToXYZ
    """
    def __init__(self, node_name: str = 'jazzy_gui_node'):
        super().__init__(node_name)

        # Create the moveit action clients to controll the joints with the sliders
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        """
        This publisher publishes the control mode of the robot
        - gui_controll_enabled = True -> Control with the GUI sliders
        - gui_controll_enabled = False -> Control with the Potentiometers
        """
        self.controll_state_publisher = self.create_publisher(Bool, 'controll_state', 10)
        self.gui_controll_enabled = Bool()
        self.gui_controll_enabled.data = True

        """
        This publisher publishes the mode of the robot
        - simulation mode -> The hardware controller (PCA9685) does not actuate the servos,
                             movement of the robot can be seen in Rviz
        - hardware mode -> Now the real robot can be actuated and will move 
        """
        self.robot_mode_publisher = self.create_publisher(String, 'robot_mode', 10)
        self.robot_mode = String()
        self.robot_mode.data = "Simulation Mode"

        #Create the service client to use the MovetoXYZ service
        self.move_client = self.create_client(MoveToXYZ, 'move_to_xyz')
        
        """
        Wait till the service is available and prepare the request
        When the Service is not available the GUI will not start 
        """
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Warte auf Service /move_to_xyz...')
        self.req = MoveToXYZ.Request()
    
        self.get_logger().info('JazzyGuiNode initialized')

class UiWindow:
    """
    Qt GUI class that:
    - Loads the .ui file
    - Connects widgets to callbacks
    - Translates GUI actions into ROS 2 communication
    """
    def __init__(self, ui_path: str, node: JazzyGuiNode):
        self.node = node
        self.ui_path = ui_path

        # Create a subscriber to receive the forward kinematics information
        self.fk_subscriber = self.node.create_subscription(PoseStamped, 'fk_info', self.update_line_edits, 10)

        # Create a subscriber to receive the potentiometer values
        self.poti_val_subscriber = self.node.create_subscription(Float64MultiArray, 'poti_angles', self.poti_angles_callback, 10)
        """
        - Create a subscriber to update the slider values when in pot controll mode
        - Prevents jumps when switching from Pot to GUI control.
        """
        self.pot_subscriber = self.node.create_subscription(JointState, 'joint_states', self.sync_sliders_to_movement, 10)

        # Load QT /.ui file generated with the QtDesigner (share/robot_gui/ui)
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

        # State variables for sequence execution
        self.sequence_counter = 1
        self.is_sequence_running = False
        self.sync_sliders = False
        self.current_seq_index = 0

        #initiate all the Widgets used in the GUI-Window
        self.init_sliders()
        self.init_mode_dropdown()
        self.init_controll_dropdown() 
        self.init_line_edits()
        self.init_pushbuttons()
        self.init_sequential_table()

        # Initiate and start a timer to run ros2 callbacks
        self.ros_spin_time = 100 #ms
        self.ros_spin_timer = QtCore.QTimer()
        self.ros_spin_timer.timeout.connect(self.spin_ros_node)
        self.ros_spin_timer.start(self.ros_spin_time)
        
    def init_sliders(self):
        """
        - Initialize joint and gripper sliders and connect callbacks.
        """

        # Get the sliders from the UI-File
        self.slider_joint_1 = self.window.findChild(QSlider, "slider_joint1")
        self.slider_joint_2 = self.window.findChild(QSlider, "slider_joint2")
        self.slider_joint_3 = self.window.findChild(QSlider, "slider_joint3")
        self.slider_gripper = self.window.findChild(QSlider, "gripper_slider")

        self.slider_arr = [self.slider_joint_1, self.slider_joint_2, self.slider_joint_3]

        # Configure joint-sliders
        for slider in self.slider_arr:
            slider.setMinimum(0)
            slider.setMaximum(180)
            slider.setValue(90)
            slider.setSingleStep(1)
            slider.valueChanged.connect(self.joint_slider_changed)

        # Configure gripper-slider
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
        """
        - Dropdown to switch between GUI and potentiometer control.
        """
        self.controll_dropdown = self.window.findChild(QComboBox, "controll_state_box")
        self.controll_dropdown.setCurrentIndex(1)
        self.controll_dropdown.currentTextChanged.connect(self.update_control_state)
    
    def init_mode_dropdown(self):
        """
        - Dropdown to select robot mode (hardware / simulation). Default = simulation
        """
        self.mode_dropdown = self.window.findChild(QComboBox, "mode_dropdown")
        self.mode_dropdown.setCurrentIndex(1)
        self.mode_dropdown.currentTextChanged.connect(self.update_robot_mode)

    def init_line_edits(self):
        """
        - Initialize text fields for current and desired Cartesian position.
        """
        self.current_pos_x = self.window.findChild(QLineEdit, "current_pos_x")
        self.current_pos_y = self.window.findChild(QLineEdit, "current_pos_y")
        self.current_pos_z = self.window.findChild(QLineEdit, "current_pos_z")

        self.current_pos_x.setText("--")
        self.current_pos_y.setText("--")
        self.current_pos_z.setText("--")

        self.desired_pos_x = self.window.findChild(QLineEdit, "desired_pos_x")
        self.desired_pos_y = self.window.findChild(QLineEdit, "desired_pos_y")
        self.desired_pos_z = self.window.findChild(QLineEdit, "desired_pos_z")

        self.desired_pos_x.setText("--")
        self.desired_pos_y.setText("--")
        self.desired_pos_z.setText("--")

    def init_pushbuttons(self):
        """
        - Initialize the pushbuttons for the save/execute sequence functionality
        """
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
        """
        - Initialize the sequential table that displays the positional und gripper sequence
        """
        self.sequential_table = self.window.findChild(QTableView, "sequential_table")

        # Set the tables format and column names
        self.sequence_model = QStandardItemModel(0, 4)
        self.sequence_model.setHorizontalHeaderLabels(
        ["Type", "X", "Y", "Z"]
        )
        self.sequential_table.setModel(self.sequence_model)

        header = self.sequential_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Fixed)
        header.setStretchLastSection(False)

        self.sequential_table.setColumnWidth(0, 104)  # Name
        self.sequential_table.setColumnWidth(1, 62)  # X
        self.sequential_table.setColumnWidth(2, 62)  # Y
        self.sequential_table.setColumnWidth(3, 62)  # Z

        self.sequential_table.verticalHeader().setVisible(False)
        self.sequential_table.setShowGrid(True)
        self.sequential_table.setAlternatingRowColors(True)
        self.sequential_table.setSelectionBehavior(QTableView.SelectRows)
        self.sequential_table.setSelectionMode(QTableView.SingleSelection)

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
        """
        - Update the robot_mode variable and publish it to the robot_mode topic
        """
        if text == "Hardware Mode":
            self.node.robot_mode.data = "Hardware Mode"
            self.node.get_logger().info("Changed Robot Mode to: Hardware Mode")
        elif text == "Simulation Mode":
            self.node.robot_mode.data = "Simulation Mode"
            self.node.get_logger().info("Changed Robot Mode to: Simulation Mode")
        
        self.node.robot_mode_publisher.publish(self.node.robot_mode)

    def update_line_edits(self, msg):
        """
        - Is executed everytime a new information is published on the fk_info topic
        - The line edits are updated everytime
        - The frequency is determined by the timer in the robot_fk node
        """
        x_mm = int(round(msg.pose.position.x * 100))
        y_mm = int(round(msg.pose.position.y * 100))
        z_mm = int(round(msg.pose.position.z * 100))

        self.current_pos_x.setText(f"{x_mm}")
        self.current_pos_y.setText(f"{y_mm}")
        self.current_pos_z.setText(f"{z_mm}")
    
    def sync_sliders_to_movement(self, msg):
        """
        - The sync is important when:
            - planning with MoveIt via drag, plan, execute
            - controlling the robot with the potentiometers
            - sending a sequence to the robot
        - It is necessary to avoid crazy jumps when switching the used control tool
        """
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

        # Update labels manually to fit the position
        self.slider_label_1.setText(f"Joint Angle 1: {deg_2} degrees")
        self.slider_label_2.setText(f"Joint Angle 2: {deg_3} degrees")
        self.slider_label_3.setText(f"Joint Angle 3: {deg_4} degrees")

    def set_sync_sliders_flag(self, flag):
        # Simple flag updater used by: send_desired_position, service_response_callback
        self.sync_sliders = flag

    def show(self):
        # Default function needed to show the GUI-Window
        self.window.show()  

    def rad_vec_to_deg_0_180(self, rad_list):
        # Convert radians in range [-pi/2 : pi/2] → [0–180°]
        return [int(round((r * 180 / pi) + 90)) for r in rad_list] 
    
    def send_position_sequence(self):
        """
        - Starts execution of a stored position / gripper sequence.
        - Initializes sequence state and triggers the first command.
        """
        if self.sequence_model.rowCount() == 0:
            return
        
        self.is_sequence_running = True
        self.current_seq_index = 0
    
        # Start execution with the first sequence entry
        self.send_desired_position(self.current_seq_index, sequence=True)

    def send_desired_position(self, pos_number=0, sequence=False):
        """
        - Sends either a single Cartesian goal or a sequence element.
        - Handles MOVE commands via service calls and GRIPPER commands via actions.
        """
        self.set_sync_sliders_flag(True)

        try:
            if sequence:
                # Read command type from sequence table
                cmd_type = self.sequence_model.item(pos_number, 0).text()  

                if cmd_type == "MOVE":
                    # Read Cartesian target (mm → cm)
                    name = self.sequence_model.item(pos_number, 0).text()
                    x_val = float(self.sequence_model.item(pos_number, 1).text()) / 100.0
                    y_val = float(self.sequence_model.item(pos_number, 2).text()) / 100.0
                    z_val = float(self.sequence_model.item(pos_number, 3).text()) / 100.0

                    self.node.get_logger().info(f"Sending position request for {name}: X={x_val}, Y={y_val}, Z={z_val}")

                elif cmd_type == "GRIPPER":
                    # Convert gripper percentage to joint position (rad)
                    val_str = self.sequence_model.item(pos_number, 1).text()
                    closure_percent = float(val_str)
                    position_rad = (closure_percent / 100.0) * (pi / 2.0)

                    goal = GripperCommand.Goal()
                    goal.command.position = position_rad

                    # Send gripper action asynchronously
                    self.node.get_logger().info(f"Sending Gripper Command: {closure_percent:.2f} % closed")
                    send_goal_future = self.node.gripper_client.send_goal_async(goal)

                    send_goal_future.add_done_callback(self.gripper_get_result_callback)
                    return

            else:
                # Read desired Cartesian position from input fields
                x_val = float(self.desired_pos_x.text()) / 100.0
                y_val = float(self.desired_pos_y.text()) / 100.0
                z_val = float(self.desired_pos_z.text()) / 100.0

                self.node.get_logger().info(f"Sende Service Anfrage: X={x_val}, Y={y_val}, Z={z_val}")

            # Fill MoveToXYZ Request
            self.node.req.x = x_val
            self.node.req.y = y_val
            self.node.req.z = z_val

            # Send the request asynchronosly so the GUI doesnt freeze
            future = self.node.move_client.call_async(self.node.req)
            
            # Define the function that is called when the service is done 
            future.add_done_callback(self.service_response_callback)

        except ValueError:
            self.node.get_logger().error("Please make sure the inputs are numbers!")

    def service_response_callback(self, future):
        """
        - Handles responses from the MoveToXYZ service.
        - Advances the sequence if active.
        """
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info(f"Erfolg: {response.message}")
                # Disable slider synchronization after delay 
                # The delay is necessary to let the sliders catch up 
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
        
        # Handle sequence continuation
        if self.is_sequence_running:
            self.current_seq_index += 1 
        
            if self.current_seq_index < self.sequence_model.rowCount():
                self.node.get_logger().info(f"Starting next position: {self.current_seq_index}")
                self.send_desired_position(self.current_seq_index, sequence=True)

            else:
                self.node.get_logger().info("Sequence finished.")
                QtCore.QTimer.singleShot(2000, lambda: self.set_sync_sliders_flag(False))
                self.node.get_logger().info(f"sync_sliders = {self.sync_sliders}")

    def gripper_get_result_callback(self, future):
        """
        - Callback executed after a gripper action finishes.
        - Continues sequence execution if active.
        """
        self.node.get_logger().info('Greifer-Aktion abgeschlossen.')

        # Handle sequence continuation
        if self.is_sequence_running:
            self.current_seq_index += 1
            if self.current_seq_index < self.sequence_model.rowCount():
                self.send_desired_position(self.current_seq_index, sequence=True)
                
            else:
                self.node.get_logger().info("Sequenz vollständig beendet.")
                self.is_sequence_running = False

    def save_current_position(self):
        """
        - Stores the current FK position as a MOVE command in the sequence table.
        - The forward kinematics node needs to be running 
        """
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
        """
        Stores the current gripper state as a GRIPPER command in the sequence.
        """
        closure_percent = (self.slider_gripper.value() - 90.0) / 90.0 * 100.0
        closure_str = str(closure_percent)

        row_items = [
            QStandardItem("GRIPPER"), # Typ
            QStandardItem(str(closure_str)),  # Val 1 (Cosure percent)
            QStandardItem("% Closed"),       # Val 2 (empty)
            QStandardItem("-")        # Val 3 (empty)
        ]
        self.sequence_model.appendRow(row_items)
        self.sequence_counter += 1

    def delete_selected_position(self):
        """
        Removes the currently selected row from the sequence table.
        """
        selection_model = self.sequential_table.selectionModel()

        if not selection_model.hasSelection():
            self.node.get_logger().warn("No row is selected")
            return

        row = selection_model.selectedRows()[0].row()
        self.sequence_model.removeRow(row)

    @QtCore.Slot()
    def spin_ros_node(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    @QtCore.Slot(int)
    def joint_slider_changed(self, value): 
        """
        - Sends joint trajectory commands when sliders are moved
        - But only when GUI control is enabled.
        """
        value_1 = float(self.slider_joint_1.value())
        value_2 = float(self.slider_joint_2.value())
        value_3 = float(self.slider_joint_3.value())
        
        # Update the labels
        self.slider_label_1.setText(f"Joint Angle 1: {value_1} degrees")
        self.slider_label_2.setText(f"Joint Angle 2: {value_2} degrees")
        self.slider_label_3.setText(f"Joint Angle 3: {value_3} degrees")

        # Convert 0-180 -> -pi/2 ... pi/2
        rad_1 = (value_1 - 90) * pi / 180
        rad_2 = (value_2 - 90) * pi / 180
        rad_3 = (value_3 - 90) * pi / 180

        if self.node.gui_controll_enabled.data:
            traj = FollowJointTrajectory.Goal()
            traj.trajectory.joint_names = ['joint_1','joint_2','joint_3']

            point = JointTrajectoryPoint()
            point.positions = [rad_1, rad_2, rad_3]
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 300_000_000  # 300 ms

            traj.trajectory.points.append(point)
            self.node.trajectory_client.send_goal_async(traj)
    
    def gripper_slider_changed(self, value):
        """
        Sends gripper command when the gripper slider changes.
        """

        # 90–180 → 0.0–1.0
        position = (value - 90) * pi / 180.0

        goal = GripperCommand.Goal()
        goal.command.position = position

        self.node.gripper_client.send_goal_async(goal)

    def poti_angles_callback(self, msg):
        """
        Wird aufgerufen, wenn der Raspberry Pi neue Winkel sendet.
        """
        # Nur ausführen, wenn Poti-Steuerung im Dropdown aktiv ist
        if not self.node.gui_controll_enabled.data:
            
            # 1. Trajektorie an den Roboter senden
            traj = FollowJointTrajectory.Goal()
            traj.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3']

            point = JointTrajectoryPoint()
            # msg.data enthält [rad_1, rad_2, rad_3] vom Pi
            point.positions = list(msg.data)
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 250_000_000  # 250ms für flüssige Bewegung

            traj.trajectory.points.append(point)
            
            # Wichtig: Zeitstempel vom PC setzen (verhindert Fehlermeldungen wegen Zeitunterschieden)
            traj.trajectory.header.stamp = self.node.get_clock().now().to_msg()

            self.node.trajectory_client.send_goal_async(traj)
    

def main(argv=None):
    argv = argv or sys.argv

    # Initialise ROS2 Content 
    rclpy.init(args=argv)

    # Create the ROS 2 node used by the GUI
    node = JazzyGuiNode()

    # Resolve path to the Qt Designer .ui file inside the package
    pkg_share = get_package_share_directory('robot_gui') 
    ui_path = Path(pkg_share) / 'ui' / 'robot_gui.ui'      

    # Exception if the .ui file cant be found
    if not ui_path.exists():
        node.get_logger().error(f"UI-Datei nicht gefunden: {ui_path}")
        rclpy.shutdown()
        return 1

    # Create Qt application instance
    app = QApplication(argv)
    ui = UiWindow(str(ui_path), node)
    ui.show()

    #Signal-Handler to close the window on Strg+C
    def signal_handler(sig, frame):
        """
        Ensures a clean shutdown when Ctrl+C is received.
        This is needed to be able to either close the window or use Ctrl+C to shutdown the application
        """
        node.get_logger().info("Ctrl+C received, closing GUI...")
        ui.window.close()          # close the window
        node.destroy_node()        # shutdown the ros2 node
        rclpy.shutdown()           # shutdown ros2 
        app.quit()                 # end the Qt Event Loop

    signal.signal(signal.SIGINT, signal_handler)

    try:
        return_code = app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return return_code

if __name__ == '__main__':
    raise SystemExit(main())