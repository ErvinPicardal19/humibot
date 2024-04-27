from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   
   humibot_teleop_pkg = get_package_share_directory("humibot_teleop")
   
   joy_params= os.path.join(humibot_teleop_pkg, "config/joy_params.yaml")
   
   start_joy_node = Node(
      package="joy",
      executable="joy_node",
      parameters=[joy_params]
   )

   start_teleop_joy = Node(
      package="teleop_twist_joy",
      executable="teleop_node",
      name="teleop_node",
      parameters=[joy_params],
      remappings=[
         ("/cmd_vel", "/diff_controller/cmd_vel_unstamped")
      ]
   )
   
   return LaunchDescription([
      start_joy_node,
      start_teleop_joy
   ])