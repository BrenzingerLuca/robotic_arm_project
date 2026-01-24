
import time
import math

# Imports needed for the Pi and the PCA
import board
import busio
from adafruit_pca9685 import PCA9685

# General ROS2 imports 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class ServoController(Node):
    """
    ROS 2 node that maps JointState positions to PWM-controlled servos
    using a PCA9685 over I2C.

    Intent:
    - Act as a hardware bridge between ROS2 joint states and real servos
    - Allow switching between simulation and hardware mode at runtime in the GUI

    Assumptions:
    - Joint positions are published in radians, range [-pi/2, +pi/2]
    - Each joint corresponds to one servo
        - Gripper = Channel 0
        - Joint 1 = Channel 1
        - Joint 2 = Channel 2 
        - Joint 3 = Channel 3
    """
    
    def __init__(self, num_servos=4, freq=50):
        super().__init__('servo_controller')
        
        """
        WHY PCA9685 + Adafruit library:
        - Offloads PWM generation from the Raspberry Pi while also generating preciser PWM stopping the servos to jitter
        - Provides stable servo timing independent of OS load
        """
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = freq

        # Physical PCA9685 channels where servos are connected 
        self.pca_channels = [0, 1, 2, 3]

        """
        Robot mode handling:
        - Robot mode is published by the GUI to the /robot_mode topic when changed 
        - The robot mode is  set to simulation in the beginning for safety reasons
        - 'Simulation Mode' disables hardware output
        - Prevents accidental servo motion when starting 
        - A subscriber is created to update the mode when it's changed
        """
        self.robot_mode = "Simulation Mode"
        self.robot_mod_sub = self.create_subscription(String, 'robot_mode', self.update_robot_mode, 10)

        # Subscribe to joint_states to receive the desired joint positions
        self.create_subscription(JointState, 'joint_states', self.pot_callback, 10)
        self.num_servos = num_servos

        # Current servo angles (degrees), initialized to neutral position
        self.angles = [90.0] * self.num_servos
        self.goal_angles = self.angles

        """
        Motion smoothing parameter:
        - Limits how fast the servo moves toward the target
        - Reduces mechanical stress and jitter
        """
        self.max_step = 1.0 
        
        #Create a timer to update the servo position (50 Hz)
        self.create_timer(0.02, self.update)
        self.get_logger().info("Servo controller initialized")


    def pot_callback(self, msg):
        """
        JointState callback.

        Converts joint positions from radians to servo angles in degrees
        and stores them as target angles.

        Design choice:
        - Mapping [-pi/2, +pi/2] → [0°, 180°]
        - Matches common servo limits and the joint state values of the GUI and MoveIt node
        """
        num_positions = len(msg.position)

        for i in range(self.num_servos):
            if i < num_positions:
                rad = msg.position[i]
                deg = math.degrees(rad) + 90.0  # -pi/2 -> 0°, pi/2 -> 180°
                self.goal_angles[i] = deg
            else:
                # Defensive warning if joint_states is incomplete
                self.get_logger().warn(f"Only ({num_positions}) potentiometers are publishing for ({self.num_servos}) servos")
  
    def angle_to_raw12(self, angle):
        """
        Converts a servo angle (degrees) into a 12-bit PCA9685 PWM value.

        Assumptions:
        - Servo pulse range: 0.5 ms to 2.5 ms
        - PCA9685 uses 12-bit resolution (0-4095)
        """
        pulse_ms = 0.5 + (angle / 180.0) * 2.0
        fraction = pulse_ms / (1000.0 / self.pca.frequency)
        return max(0, min(int(fraction * 4095), 4095))

    def update(self):
        if self.robot_mode == "Simulation Mode":
            return
        
        for i in range(self.num_servos):
            # Gradual convergence toward target angle
            angle_diff = self.goal_angles[i] - self.angles[i]

            if abs(angle_diff) < self.max_step:
                self.angles[i] = self.goal_angles[i]
            # Smooth motion instead of hard step
            else:
                self.angles[i] += angle_diff * 0.2
        
            # Convert angle to PWM and send to hardware
            raw = self.angle_to_raw12(self.angles[i])
            self.pca.channels[self.pca_channels[i]].duty_cycle = raw << 4

    def update_robot_mode(self, msg):
        """
        Updates robot mode at runtime.

        WHY this exists:
        - To be able to switch the robot mode in the GUI 
        """
        self.robot_mode = msg.data
        if self.robot_mode == "Simulation Mode":
            self.get_logger().info("Robot Mode changed to Simulation Mode: Servo controll disabled")
        else:
            self.get_logger().info("Robot Mode changed to Hardware Mode: Servo controll enabled.")

def main(args=None):
    """
    Standard ROS 2 node lifecycle.

    Important cleanup:
    - PCA9685 is deinitialized explicitly to release I2C resources
    """
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pca.deinit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()