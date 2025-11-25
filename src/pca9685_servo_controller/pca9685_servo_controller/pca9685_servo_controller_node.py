#!/usr/bin/env python3
import time
import math

import board
import busio
from adafruit_pca9685 import PCA9685

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class ServoController(Node):
    def __init__(self, num_servos=3, freq=50):
        super().__init__('servo_controller')
        
        #using the adafruit_pca9685 lib to connect the pca to the pi via I2C
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = freq

        #defining the channels of the pca where the servos are connected 
        self.pca_channels = [0, 1, 2]

        #create a subscriber that subscribes to the /joint_states topic
        self.create_subscription(JointState, 'joint_states', self.pot_callback, 10)
        self.num_servos = num_servos
        #Set the initial servo angles to 90 
        self.angles = [90.0] * self.num_servos
        self.goal_angles = self.angles

        #defines how much the servo can move during one update (1 degree every 0.02 seconds)
        self.max_step = 1.0 
        
        #Create a timer to update the servo position every 0.05 s
        self.create_timer(0.02, self.update)
        self.get_logger().info("Init done")

    def pot_callback(self, msg):
        num_positions = len(msg.position)

        for i in range(self.num_servos):
            if i < num_positions:
                # Setze den Zielwinkel für diesen Servo
                # Radiant -> Grad und verschiebe von [-90,90] auf [0,180]
                rad = msg.position[i]
                deg = math.degrees(rad) + 90.0  # -pi/2 -> 0°, pi/2 -> 180°
                self.goal_angles[i] = deg
            else:
                # Wenn weniger Werte als Servos, behalte den aktuellen Winkel
                self.get_logger().warn(f"Only ({num_positions}) potentiometers are publishing for ({self.num_servos}) servos")
        
    def angle_to_raw12(self, angle):
        pulse_ms = 0.5 + (angle / 180.0) * 2.0
        fraction = pulse_ms / (1000.0 / self.pca.frequency)
        return max(0, min(int(fraction * 4095), 4095))

    def update(self):
        for i in range(self.num_servos):
            # Schrittweise Annäherung an den Zielwinkel
            angle_diff = self.goal_angles[i] - self.angles[i]

            if abs(angle_diff) < self.max_step:
                self.angles[i] = self.goal_angles[i]

            else:
                self.angles[i] += angle_diff * 0.2
        
            #calculate the pwm value and set it 
            raw = self.angle_to_raw12(self.angles[i])
            self.pca.channels[self.pca_channels[i]].duty_cycle = raw << 4

def main(args=None):
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