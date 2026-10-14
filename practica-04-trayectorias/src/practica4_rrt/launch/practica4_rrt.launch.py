import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure(context):
    parameters = dict(
        seed=int(LaunchConfiguration('seed').perform(context)),
        execute=LaunchConfiguration('execute').perform(context).lower() == 'true',
        use_sim_time=LaunchConfiguration('use_sim_time').perform(context).lower() == 'true',
        seconds_per_meter=float(LaunchConfiguration('seconds_per_meter').perform(context)),
        tracking_kp=float(LaunchConfiguration('tracking_kp').perform(context)),
        vertical_kp=float(LaunchConfiguration('vertical_kp').perform(context)),
        final_position_tolerance=float(
            LaunchConfiguration('final_position_tolerance').perform(context)
        ),
        results_dir=LaunchConfiguration('results_dir').perform(context),
    )
    return [Node(package='practica4_rrt', executable='rrt_navigation', output='screen',
                 parameters=[parameters])]


def generate_launch_description():
    share = get_package_share_directory('practica4_rrt')
    bringup = get_package_share_directory('sjtu_drone_bringup')
    defaults = dict(execute='true', gazebo='true',
                    use_gui='true', rviz='false', use_sim_time='true',
                    seed='22', seconds_per_meter='2.0', tracking_kp='3.5',
                    vertical_kp='2.0', final_position_tolerance='0.03',
                    results_dir=os.path.expanduser('~/ros2_ws/results/practica4'))
    actions = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    actions += [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sjtu_drone_gazebo.launch.py')),
        condition=IfCondition(LaunchConfiguration('gazebo')),
        launch_arguments={'world': os.path.join(share, 'worlds', 'rrt_obstacles.world'),
                          'use_gui': LaunchConfiguration('use_gui'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}.items()),
        OpaqueFunction(function=configure),
        Node(package='rviz2', executable='rviz2',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share, 'rviz', 'rrt.rviz')],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])]
    return LaunchDescription(actions)
