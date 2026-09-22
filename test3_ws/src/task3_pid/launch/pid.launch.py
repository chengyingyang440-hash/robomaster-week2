from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="task3_pid",
            executable="motor_simulator",
            name="motor_simulator",
            output="screen"
        ),

        Node(
            package="task3_pid",
            executable="speed_controller",
            name="speed_controller",
            output="screen",
        ),

    ])