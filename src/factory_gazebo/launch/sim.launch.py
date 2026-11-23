"""Start Gazebo with the factory world, the ROS bridge and (optionally) the arm.

    ros2 launch factory_gazebo sim.launch.py               # GUI + arm
    ros2 launch factory_gazebo sim.launch.py gui:=false    # headless server
    ros2 launch factory_gazebo sim.launch.py with_robot:=false
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz_pkg = FindPackageShare('factory_gazebo')
    desc_pkg = FindPackageShare('factory_description')
    bringup_pkg = FindPackageShare('factory_bringup')
    gui = LaunchConfiguration('gui')
    with_robot = LaunchConfiguration('with_robot')

    world = PathJoinSubstitution([gz_pkg, 'worlds', 'factory.sdf'])
    gz_args = PythonExpression([
        "'-r ' + ('' if '", gui, "' == 'true' else '-s --headless-rendering ') + '", world, "'"])

    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
        parameters=[{'config_file': PathJoinSubstitution([gz_pkg, 'config', 'bridge.yaml']),
                     'use_sim_time': True}],
        output='screen')

    # names are lost by the bridge's Pose_V conversion, so poses go through this relay
    pose_relay = Node(package='factory_control', executable='pose_relay_node',
                      parameters=[{'use_sim_time': True}], output='screen')

    controllers = PathJoinSubstitution([bringup_pkg, 'config', 'controllers.yaml'])
    robot_description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([desc_pkg, 'urdf', 'arm.urdf.xacro']),
        ' use_sim:=true controllers_file:=', controllers]), value_type=str)

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        condition=IfCondition(with_robot))

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', 'factory', '-topic', 'robot_description', '-name', 'factory_arm'],
        output='screen', condition=IfCondition(with_robot))

    def spawner(*names):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[*names, '--controller-manager', '/controller_manager',
                               '--param-file', controllers],
                    parameters=[{'use_sim_time': True}], condition=IfCondition(with_robot))

    jsb = spawner('joint_state_broadcaster')
    arm_and_gripper = spawner('arm_controller', 'gripper_controller')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('with_robot', default_value='true'),
        gazebo, bridge, pose_relay, rsp, spawn_robot,
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm_and_gripper])),
    ])
