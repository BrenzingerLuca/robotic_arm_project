from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Potentiometer Reader (Reads potentiometer values from the ADC)
        # It publishes the joint angles to the 'poti_angles' topic
        Node(
            package='potentiometer_reader',
            executable='poti_reader',
            name='poti_reader_node',
            output='screen'
        ),
        
        # 2. PCA9685 Servo Controller (Receives commands and controls hardware)
        # This node is responsible for actuating the physical servos
        Node(
            package='pca9685_servo_controller',
            executable='servo_pca_controller',
            name='servo_pca_controller_node',
            output='screen'
        )
    ])