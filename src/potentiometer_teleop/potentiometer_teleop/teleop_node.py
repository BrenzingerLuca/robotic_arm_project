import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math

# Nachrichten & Actions
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Bool

# I2C Kommunikation
import smbus2

class PotentiometerTeleopNode(Node):
    def __init__(self):
        super().__init__('potentiometer_teleop_node')

        # Action Client für die Robotersteuerung
        self.trajectory_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        # Subscribes to control-state (Nur senden, wenn GUI-Control aus ist)
        self.gui_controll_state_subscriber = self.create_subscription(
            Bool, 'controll_state', self.controll_state_callback, 10)
        
        # Hardware Setup
        self.num_servos = 3
        try:
            self.bus = smbus2.SMBus(1)
            self.adc_address = 0x4b
            self.channel_bytes = [0x84, 0xC4, 0x94]
        except Exception as e:
            self.get_logger().error(f"Konnte I2C Bus nicht öffnen: {e}")

        # Speicher für die letzten gesendeten Winkel (Deadband-Filter)
        self.last_angles = [0.0] * self.num_servos
        self.threshold = 0.05  # Rausch-Schwellwert in Radian (~3 Grad)

        self.timer = None
        self.get_logger().info("Potentiometer Teleop Node bereit. Warte auf controll_state=False...")

    def read_channel(self, channel):
        try:
            control_byte = self.channel_bytes[channel]
            self.bus.write_byte(self.adc_address, control_byte)
            time.sleep(0.005) 
            return self.bus.read_byte(self.adc_address)
        except Exception as e:
            self.get_logger().error(f"I2C Lesefehler auf Kanal {channel}: {e}")
            return None

    def send_trajectory_goal(self, angles):
        if not self.trajectory_client.wait_for_server(timeout_sec=0.1):
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3']

        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start.nanosec = 200_000_000 # 200ms für geschmeidige Bewegung

        goal_msg.trajectory.points.append(point)
        self.trajectory_client.send_goal_async(goal_msg)

    def timer_callback(self):
        adc_values = []
        for i in range(self.num_servos):
            val = self.read_channel(i)
            if val is None: return 
            adc_values.append(val)

        # Umrechnung 0-255 -> -pi/2 bis pi/2
        current_angles = [((v / 255.0) * math.pi) - (math.pi / 2) for v in adc_values]

        # Prüfen, ob Bewegung groß genug ist (Filter gegen Rauschen)
        diff = sum(abs(curr - last) for curr, last in zip(current_angles, self.last_angles))
        
        if diff > self.threshold:
            self.send_trajectory_goal(current_angles)
            self.last_angles = current_angles

    def controll_state_callback(self, msg: Bool):
        if msg.data is True: # GUI aktiv
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
                self.get_logger().info("Poti-Steuerung PAUSIERT (GUI übernimmt).")
        else: # GUI inaktiv -> Poti übernimmt
            if self.timer is None:
                self.timer = self.create_timer(0.1, self.timer_callback)
                self.get_logger().info("Poti-Steuerung AKTIV.")

def main(args=None):
    rclpy.init(args=args)
    node = PotentiometerTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()