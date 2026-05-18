import os
import xacro

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    arm_id_parameter_name = 'arm_id'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'
    enable_friction_compensation_parameter_name = 'enable_friction_compensation'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    arm_id = LaunchConfiguration(arm_id_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    enable_friction_compensation = LaunchConfiguration(enable_friction_compensation_parameter_name)
    

    pkg_path = get_package_share_directory('joint_impedance_control')
    # Execute the set_load.sh script
    set_load = ExecuteProcess(
        cmd=[os.path.join(pkg_path, 'launch', 'set_load.sh')],  # Reference to the shell script in the same folder
        output='screen',
    )

    set_torque_limits = ExecuteProcess(
        cmd=[os.path.join(pkg_path, 'launch', 'set_force_torque_limits.sh')],  # Reference to the shell script in the same folder
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            arm_id_parameter_name,
            default_value='fr3',
            description='ID of the type of arm used. Supported values: fer, fr3, fp3'),
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            default_value='192.168.1.200',
            description='Hostname or IP address of the robot.'),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='false',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'),
        DeclareLaunchArgument(
            enable_friction_compensation_parameter_name,
            default_value='false',
            description='Run the external friction compensation node.'),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description="Fake sensor commands. Only valid when '{}' is true".format(
                use_fake_hardware_parameter_name)),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        # Include the main Franka bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'])]),
            launch_arguments={'robot_ip': robot_ip,
                              'robot_type': arm_id,
                              'load_gripper': load_gripper,
                              'use_fake_hardware': use_fake_hardware,
                              'fake_sensor_commands': fake_sensor_commands,
                              'use_rviz': use_rviz
                              }.items(),
        ),

        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare('franka_fr3_moveit_config'),
                            'launch',
                            'move_group.launch.py',
                        ]
                    )
                ]
            ),
            launch_arguments={
                robot_ip_parameter_name: robot_ip,
                arm_id_parameter_name: arm_id,
                load_gripper_parameter_name: load_gripper,
                use_fake_hardware_parameter_name: 'true',
                fake_sensor_commands_parameter_name: fake_sensor_commands,
                use_rviz_parameter_name: use_rviz,
            }.items(),
        ),
        
        
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_impedance_ik_controller'],
            output='screen',
        ),
        Node(
            package='joint_impedance_control',
            executable='friction_compensation_node',
            name='friction_compensation_node',
            output='screen',
            condition=IfCondition(enable_friction_compensation),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
        )
    ])
