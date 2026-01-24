import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import smbus2
import time
import math

class PotiPublisher(Node):
    def __init__(self):
        super().__init__('poti_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'poti_angles', 10)
        
        # I2C Setup
        try:
            self.bus = smbus2.SMBus(1)
            self.adc_address = 0x4b
            self.channel_bytes = [0x84, 0xC4, 0x94]
        except Exception as e:
            self.get_logger().error(f"I2C Fehler: {e}")

        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz

    def read_channel(self, channel):
        try:
            self.bus.write_byte(self.adc_address, self.channel_bytes[channel])
            time.sleep(0.005)
            return self.bus.read_byte(self.adc_address)
        except:
            return None

    def timer_callback(self):
        values = []
        for i in range(3):
            val = self.read_channel(i)
            if val is not None:
                # Umrechnung in Radian
                rad = ((val / 255.0) * math.pi) - (math.pi / 2)
                values.append(rad)
        
        if len(values) == 3:
            msg = Float64MultiArray()
            msg.data = values
            self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = PotiPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()