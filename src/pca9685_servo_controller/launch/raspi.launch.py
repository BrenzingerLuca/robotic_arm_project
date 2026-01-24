from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Die neue Teleop-Node (Liest Poti-Werte und sendet Trajectories)
        Node(
            package='potentiometer_teleop',
            executable='teleop_node',
            name='potentiometer_teleop_node',
            output='screen'
        ),
        
        # 2. Der PCA9685 Servo Controller (Empfängt Befehle und steuert Hardware)
        Node(
            package='pca9685_servo_controller',
            executable='servo_pca_controller',
            name='servo_pca_controller_node',
            output='screen'
        )
    ])