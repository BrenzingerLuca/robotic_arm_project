import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Library for I2C communication -> Reading the ADC value
import smbus2

class potentiometer_pub(Node):

    def __init__(self):
        super().__init__('potentiometer_pub')

        self.publisher_ = self.create_publisher(Float32, 'servo_angle', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bus = smbus2.SMBus(1)  # Use I2C bus 1
        self.adc_address = 0x4b  # Adress of the ADC 

        

    def timer_callback(self):
        # Read potentiometer value from ADC via I2C
        self.bus.write_byte(0x4b, 0x84)
        # The ADC returns a value between 0 and 255
        adc_value = self.bus.read_byte(0x4b)

        # Map the ADC value (0-255) to servo angle (0-180)
        servo_angle = (adc_value / 255.0) * 180.0

        msg = Float32()
        msg.data = servo_angle

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    potentiometer_publisher = potentiometer_pub()
    rclpy.spin(potentiometer_publisher)

    potentiometer_publisher.destroy_node()
    rclpy.shutdown()
