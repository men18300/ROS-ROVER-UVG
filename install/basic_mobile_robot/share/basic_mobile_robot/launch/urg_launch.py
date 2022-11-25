import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

   param_file = os.path.join(get_package_share_directory('basic_mobile_robot'), 'params', 'urg.yaml')
   return LaunchDescription([
   Node(
   name= 'urg_node',
   package = 'urg_node',
   executable = 'urg_node_driver',
  # output = 'screen',
   parameters = [param_file],
   remappings=[('scan','scan'),]
   )
   ])
