import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import Servo

#Optional kann man noch pigpio oder ähnliche bibs einfügen für präzisere pwm
#from gpiozero.pins.pigpio import PiGPIOFactory # For better PWM control

# Important: Ensure pigpiod is running
# sudo systemctl start pigpiod
# sudo systemctl enable pigpiod

class ServoController(Node):

    def __init__(self):
        super().__init__('servo_controller')
        #self.factory = PiGPIOFactory() # Use pigpio for better accuracy
        self.servo_pin = self.declare_parameter('servo_pin', 18).value # Default to GPIO 18 (PWM0)
        
        # Adjust min_pulse_width and max_pulse_width for your specific servo
        # These values are typical for SG90 servos, but may need fine-tuning
        self.servo = Servo(self.servo_pin, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
        
        self.subscription = self.create_subscription(
            Float32,
            'servo_angle',
            self.angle_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info(f'Servo Controller Node started on GPIO {self.servo_pin}. Ready to receive angles on /servo_angle topic.')

    def angle_callback(self, msg):
        angle_degrees = msg.data
        self.get_logger().info(f'Received angle: {angle_degrees:.2f} degrees')

        # Map angle (0-180) to servo value (-1 to 1)
        # -1 corresponds to min_pulse_width, 1 to max_pulse_width
        # Assuming 0 degrees = min_pulse_width, 180 degrees = max_pulse_width
        if 0 <= angle_degrees <= 180:
            servo_value = (angle_degrees / 180.0) * 2 - 1
            self.servo.value = servo_value
            self.get_logger().info(f'Set servo value to: {servo_value:.2f}')
        else:
            self.get_logger().warn(f'Angle {angle_degrees:.2f} out of range (0-180 degrees). Not moving servo.')

def main(args=None):
    rclpy.init(args=args)
    servo_controller = ServoController()
    rclpy.spin(servo_controller)
    servo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
