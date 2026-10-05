from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hardware_system',
            executable='emulator_node',
            name='emulator_node',
            parameters=[{
                'temperatura': 55,
                'motores_listos': True,
                'debug_msg': 'Ejecutando desde archivo launch'
            }]
        ),
        Node(
            package='hardware_system',
            executable='monitor_node',
            name='monitor_node',
            parameters=[{
                'temp_limite': 60
            }]
        ),
        Node(
            package='hardware_system',
            executable='operator_node',
            name='operator_node'
        )
    ])
