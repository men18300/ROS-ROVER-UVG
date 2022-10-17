import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
 
    pkg_share = FindPackageShare(package='basic_mobile_robot').find('basic_mobile_robot')
    default_launch_dir = os.path.join(pkg_share, 'launch')
    default_model_path = os.path.join(pkg_share, 'models/basic_mobile_bot_v2.urdf')
    robot_name_in_urdf = 'basic_mobile_bot'
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    robot_slam_file_path = os.path.join(pkg_share, 'params/mapper_params_online_sync.yaml') 
    
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')   
    
    # Declare the launch arguments  
    declare_model_path_cmd = DeclareLaunchArgument(
      name='model', 
      default_value=default_model_path, 
      description='Absolute path to robot urdf file') 

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')


    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation/Gazebo clock')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': Command(['xacro ', model])}],
        arguments=[default_model_path])
    
    start_sync_slam_toolbox_node = Node(
        parameters=[robot_slam_file_path,
          {'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        node_executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()
  # Declare the launch options
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd) 
    ld.add_action(declare_use_sim_time_argument)
    
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_sync_slam_toolbox_node)

    return ld
