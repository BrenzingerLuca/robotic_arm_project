#!/usr/bin/env python3
import time
import board
import busio
from adafruit_pca9685 import PCA9685
import rclpy
from rclpy.node import Node

class ServoOscillator(Node):
    def __init__(self, channel=0, freq=50):
        super().__init__('servo_oscillator')
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = freq

        self.channel = channel
        self.angle_min = 45.0
        self.angle_max = 135.0
        self.period = 0.2          # Sekunden pro Bewegung (kleiner = schneller)
        self.pause = 2.0           # Sekunden Pause an Endpunkten
        self.direction = 1
        self.angle = self.angle_min
        self.last_update = time.time()
        self.wait_until = 0.0

        self.create_timer(0.02, self.update)

    def angle_to_raw12(self, angle):
        pulse_ms = 1.0 + (angle / 180.0)
        fraction = pulse_ms / (1000.0 / self.pca.frequency)
        return max(0, min(int(fraction * 4095), 4095))

    def update(self):
        now = time.time()
        if now < self.wait_until:
            return

        step = (self.angle_max - self.angle_min) * 0.02 / self.period
        self.angle += self.direction * step

        if self.angle >= self.angle_max:
            self.angle = self.angle_max
            self.direction = -1
            self.wait_until = now + self.pause
        elif self.angle <= self.angle_min:
            self.angle = self.angle_min
            self.direction = 1
            self.wait_until = now + self.pause

        raw = self.angle_to_raw12(self.angle)
        self.pca.channels[self.channel].duty_cycle = raw << 4

def main(args=None):
    rclpy.init(args=args)
    node = ServoOscillator()
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