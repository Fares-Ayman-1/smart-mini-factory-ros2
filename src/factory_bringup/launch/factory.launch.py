"""Whole cell: Gazebo + arm + conveyor I/O + spawner + vision + supervisor + logger.

    ros2 launch factory_bringup factory.launch.py            # Gazebo GUI + RViz
    ros2 launch factory_bringup factory.launch.py gui:=false rviz:=false
    ros2 launch factory_bringup factory.launch.py max_products:=50 seed:=42   # repeatable test run

Run the calibration once first (writes config/calibration.yaml):
    ros2 launch factory_gazebo sim.launch.py with_robot:=false
    ros2 run factory_control calibrate_node --ros-args -p use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cfg = PathJoinSubstitution([FindPackageShare('factory_bringup'), 'config', 'cell.yaml'])

    def node(executable, name, extra=None, **kw):
        return Node(package='factory_control', executable=executable, name=name,
                    parameters=[cfg, {'use_sim_time': True, **(extra or {})}], output='screen', **kw)

    sim = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('factory_gazebo'), 'launch', 'sim.launch.py']),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items())

    app = [
        node('conveyor_io_node', 'conveyor_io'),
        node('arm_node', 'arm'),
        node('vision_node', 'vision'),
        node('logger_node', 'item_logger'),
        node('supervisor_node', 'supervisor'),
        node('spawner_node', 'spawner', {
            'max_products': ParameterValue(LaunchConfiguration('max_products'), value_type=int),
            'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int)}),
    ]
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', PathJoinSubstitution(
                    [FindPackageShare('factory_description'), 'rviz', 'view.rviz'])],
                parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz')),
                # RViz's renderer needs X11; under WSLg Qt otherwise picks Wayland and RViz crashes
                additional_env={'QT_QPA_PLATFORM': 'xcb'})

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('max_products', default_value='0', description='0 = unlimited'),
        DeclareLaunchArgument('seed', default_value='0', description='0 = random'),
        sim,
        # give Gazebo and the controllers time to come up
        TimerAction(period=20.0, actions=app + [rviz]),
    ])
