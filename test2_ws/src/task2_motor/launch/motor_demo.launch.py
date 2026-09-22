from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="task2_motor",
            executable="motor_simulator",
            name="motor_simulator",
            output="screen"
        ),

        Node(
            package="task2_motor",
            executable="torque_source",
            name="torque_source",
            output="screen",
        ),

    ])