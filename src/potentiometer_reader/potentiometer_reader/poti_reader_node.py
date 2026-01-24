import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import smbus2
import time
import math

class PotiPublisher(Node):
    """
    ROS 2 Node that reads analog values from an ADC via I2C 
    and publishes them as joint angles in radians.
    """
    def __init__(self):
        super().__init__('poti_publisher')

        # Publisher for the converted potentiometer angles
        self.publisher_ = self.create_publisher(Float64MultiArray, 'poti_angles', 10)
        
        # I2C Hardware Configuration
        try:
            self.bus = smbus2.SMBus(1) # Use I2C bus 1 (standard on Raspberry Pi)
            self.adc_address = 0x4b    # I2C address of the ADC chip
            # Control bytes to select channels 0, 1, and 2 on the ADC
            self.channel_bytes = [0x84, 0xC4, 0x94]
        except Exception as e:
            self.get_logger().error(f"I2C Fehler: {e}")

        # Timer to trigger the ADC reading loop at 10Hz (every 100ms)
        self.timer = self.create_timer(0.1, self.timer_callback) 
        self.get_logger().info("Potentiometer Reader Node started.")

    def read_channel(self, channel):
        """
        Reads a single raw byte from the specified ADC channel.
        :param channel: Index (0-2) corresponding to self.channel_bytes
        :return: Integer value (0-255) or None if communication fails
        """
        try:
            # Write the control byte to the ADC to select the channel
            self.bus.write_byte(self.adc_address, self.channel_bytes[channel])
            # Short delay for the ADC to process the request
            time.sleep(0.005)
            # Read the resulting 8-bit value
            return self.bus.read_byte(self.adc_address)
        except Exception as e:
            self.get_logger().error(f"I2C Read Error on channel {channel}: {e}")
            return None

    def timer_callback(self):
        """
        Main loop: reads all 3 channels, converts them to radians, 
        and publishes the result.
        """
        values = []
        for i in range(3):
            val = self.read_channel(i)
            if val is not None:
                # Convert 8-bit raw value (0-255) to radians (-pi/2 to pi/2)
                # Formula: (value / max) * range - offset
                rad = ((val / 255.0) * math.pi) - (math.pi / 2)
                values.append(rad)
        
        # Only publish if all three channels were read successfully
        if len(values) == 3:
            msg = Float64MultiArray()
            msg.data = values
            self.publisher_.publish(msg)
            # Optional: Log the angles for debugging
            # degrees = [round(math.degrees(a), 1) for a in values]
            # self.get_logger().info(f"Published Angles (deg): {degrees}")

def main():
    rclpy.init()
    node = PotiPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()