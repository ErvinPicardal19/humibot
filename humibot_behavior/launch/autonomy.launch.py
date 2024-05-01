import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
   
   humibot_behavior_pkg = get_package_share_directory("humibot_behavior")
   
   start_autonomy_node = Node(
      package="humibot_behavior",
      executable="autonomy_node",
      name="autonomy_node",
      parameters=[
         {
            "location_file" : os.path.join(humibot_behavior_pkg, "config/locations.yaml")
         }
      ]
   )
   
   return LaunchDescription([
      start_autonomy_node
   ])