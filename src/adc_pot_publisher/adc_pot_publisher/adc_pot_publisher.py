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
    """
    PotentiometerPublisher

    Intent:
    - Read analog potentiometer values via an I2C ADC
    - Convert them to joint angles
    - Publish them as sensor_msgs/JointState

    Design assumptions:
    - An external ADC is connected via I2C (bus 1)
    - Potentiometers are mapped linearly to joint angles
    - This node is disabled when GUI control is active
    """
    def __init__(self, num_servos=3):
        super().__init__('potentiometer_pub')

        # Publishes joint positions for the robot (used by controllers / state publisher)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # Timer is created dynamically depending on control mode
        self.timer = None

        # Subscribes to control-state topic to enable/disable potentiometer input
        self.gui_controll_state_subscriber = self.create_subscription(Bool, 'controll_state', self.controll_state_callback, 10)
        self.num_servos = num_servos

        # Initialize I2C bus (bus 1 is standard on Raspberry Pi)
        self.bus = smbus2.SMBus(1)  
        # I2C address of the ADC
        self.adc_address = 0x4b  
        # Control bytes selecting ADC channels (device-specific)
        self.channel_bytes = [0x84, 0xC4, 0x94] 
        
    def read_channel(self, channel):
        """
        Reads a single ADC channel.
        - ADC requires a control byte write before each read
        - Small delay is needed for conversion to complete

        Returns:
        - Integer value in range [0, 255]
        """
        control_byte = self.channel_bytes[channel]  
        self.bus.write_byte(self.adc_address, control_byte)
        # ADC conversion delay
        time.sleep(0.005)  

        value = self.bus.read_byte(self.adc_address)
        return value

    def timer_callback(self):
        """
        Periodic callback that:
        - Reads all potentiometers
        - Maps values to joint angles
        - Publishes a JointState message
        """
        adc_values = [self.read_channel(i) for i in range(self.num_servos)]

        # Map 0-255 -> 0-180 degrees
        angles = [((v / 255.0) * 3.14) - (3.14 / 2) for v in adc_values]

        # Fixed placeholder value for gripper position, because there are only 3 potentiometers
        gripper_pos = 0.1

        # Construct JointState message
        msg = JointState()

        # Joint names must exactly match the URDF (robot_description/urdf/robot.urdf.xacro)
        msg.name = ['gripper'] + [f'joint_{i+1}' for i in range(self.num_servos)]
        msg.position = [gripper_pos] + angles
        msg.header.stamp = self.get_clock().now().to_msg()
        
        #Publish the JointStates
        self.publisher_.publish(msg)
    
    def controll_state_callback(self, msg: Bool):
        """
        Enables or disables potentiometer publishing depending on control mode.

        Control logic:
        - GUI-Control (True): Stop publishing potentiometer values
        - GUI-Control (False): Start periodic publishing
        """
        
        if msg.data is True:
            if self.timer is not None:
                self.timer.cancel()
                self.get_logger().info("Timer gestoppt (GUI-Control aktiviert).")
        else:
            if self.timer is None or self.timer.is_canceled():
                self.timer = self.create_timer(0.1, self.timer_callback)
                self.get_logger().info("Timer gestartet (Potentiometer-Control aktiviert).")


def main(args=None):
    rclpy.init(args=args)

    node = PotentiometerPublisher(num_servos=3)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
