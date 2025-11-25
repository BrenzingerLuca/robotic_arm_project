import rclpy
import time
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

# Library for I2C communication -> Reading the ADC value
import smbus2

class PotentiometerPublisher(Node):

    def __init__(self, num_servos=3):
        super().__init__('potentiometer_pub')

        self.publisher_ = self.create_publisher(JointState, 'joint_angles', 10)
        self.timer = None

        self.gui_controll_state_subscriber = self.create_subscription(Bool, 'controll_state', self.controll_state_callback, 10)
        self.num_servos = num_servos
        self.bus = smbus2.SMBus(1)  # Use I2C bus 1
        self.adc_address = 0x4b  # Adress of the ADC 
        self.channel_bytes = [0x84, 0xC4, 0x94]  # Kanal 0, 1, 2
        
    def read_channel(self, channel):
        #Liest einen ADC-Kanal (0-3) aus und gibt 0-255 zurück
        control_byte = self.channel_bytes[channel] # Kanal auswählen 
        self.bus.write_byte(self.adc_address, control_byte)
        time.sleep(0.005)  # kleine Verzögerung
        value = self.bus.read_byte(self.adc_address)
        return value

    def timer_callback(self):
        # Lese alle Kanäle 0..num_servos-1
        adc_values = [self.read_channel(i) for i in range(self.num_servos)]

        # Map 0-255 auf 0-180 Grad
        angles = [((v / 255.0) * math.pi) - (math.pi / 2) for v in adc_values]

        # JointState Nachricht erstellen
        msg = JointState()
        msg.name = [f'servo_{i}' for i in range(self.num_servos)]
        msg.position = angles
        msg.header.stamp = self.get_clock().now().to_msg()
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing angles: {angles}')
    
    def controll_state_callback(self, msg: Bool):
        if msg.data is True:
            # GUI-Control aktiviert → Potentiometer stoppen
            if self.timer is not None:
                self.timer.cancel()
                self.get_logger().info("Timer gestoppt (GUI-Control aktiviert).")
        else:
            # Potentiometer-Control aktiviert → Timer wieder starten
            if self.timer is None or self.timer.is_canceled():
                self.timer = self.create_timer(0.1, self.timer_callback)
                self.get_logger().info("Timer gestartet (Potentiometer-Control aktiviert).")


def main(args=None):
    rclpy.init(args=args)

    node = PotentiometerPublisher(num_servos=3)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
