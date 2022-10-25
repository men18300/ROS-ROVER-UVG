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
    robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 
    robot_slam_file_path = os.path.join(pkg_share, 'params/mapper_params_online_sync.yaml') 
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    
    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')   
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Declare the launch arguments  
    declare_model_path_cmd = DeclareLaunchArgument(
      name='model', 
      default_value=default_model_path, 
      description='Absolute path to robot urdf file') 

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='true',
        description='Whether to start the robot state publisher')


    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
        
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RVIZ')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

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

  # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}])

    #### tf2 static transforms

    ## tf2 - base_footprint to map
    node_tf2_fp2map = Node(
        name='tf2_ros_fp_map',
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'odom', 'base_footprint'], 
        )
        
   # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])             
        
    ld = LaunchDescription()
  # Declare the launch options
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd) 
    ld.add_action(declare_use_robot_state_pub_cmd) 
    ld.add_action(declare_use_sim_time_argument)
    

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_robot_localization_cmd)
    #ld.add_action(node_tf2_fp2map)   
    ld.add_action(start_sync_slam_toolbox_node)
    ld.add_action(start_rviz_cmd)
    


    return ld
