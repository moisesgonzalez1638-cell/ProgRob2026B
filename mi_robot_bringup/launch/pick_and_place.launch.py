import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    desc = get_package_share_directory('my_robot_description')
    rviz_cfg = os.path.join(get_package_share_directory('mi_robot_bringup'), 'config', 'cell.rviz')
    gz = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(desc, 'launch', 'gazebo.launch.py')))
    rviz = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg])
    vis = Node(package='mi_robot_vision', executable='vision_node', output='screen')
    fsm = Node(package='mi_robot_task', executable='task_fsm_node', output='screen')
    return LaunchDescription([gz, rviz, TimerAction(period=6.0, actions=[vis]), TimerAction(period=8.0, actions=[fsm])])
