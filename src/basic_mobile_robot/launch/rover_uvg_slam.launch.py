import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  param_file = os.path.join(get_package_share_directory('basic_mobile_robot'), 'params', 'urg.yaml')
   
  # Start urg node for LIDAR scans  
  start_urg_node_cmd = Node(
    name= 'urg_node',
    package = 'urg_node',
    executable = 'urg_node_driver',
    # output = 'screen',
    parameters = [param_file],
    remappings=[('scan','base_scan'),])
   
   # Start comunnication with ESP32 on Rover UVG
  start_com_rover_cmd = Node(
    package='basic_mobile_robot',
    name= 'hardware_arduino_wheels',
    executable= "hardware_arduino_wheels.py",
    output='screen')   

  ld = LaunchDescription()
    
  ld.add_action(start_urg_node_cmd)
  ld.add_action(start_com_rover_cmd)
    
  return ld
