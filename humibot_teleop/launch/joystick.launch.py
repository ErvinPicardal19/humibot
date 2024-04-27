from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   
   humibot_teleop_pkg = get_package_share_directory("humibot_teleop")
   
   joy_params= os.path.join(humibot_teleop_pkg, "config/joy_params.yaml")
   
   use_sim_time = LaunchConfiguration("use_sim_time")
   declare_use_sim_time = DeclareLaunchArgument(
      name="use_sim_time",
      default_value="False",
      description="Use Gazebo clock if True"
   )
   
   start_joy_node = Node(
      package="joy",
      executable="joy_node",
      parameters=[joy_params, {"use_sim_time": use_sim_time}]
   )

   start_teleop_joy = Node(
      package="teleop_twist_joy",
      executable="teleop_node",
      name="teleop_node",
      parameters=[joy_params, {"use_sim_time": use_sim_time}],
      remappings=[
         ("/cmd_vel", "/cmd_vel_joy")
      ]
   )

   # start_twist_stamper = Node(
   #    package="twist_stamper",
   #    executable="twist_stamper",
   #    parameters=[{"use_sim_time": use_sim_time}],
   #    remappings=[
   #       ("/cmd_vel_in", "/cmd_vel"),
   #       ("/cmd_vel_out", "/cmd_vel_joy")
   #    ]
   # )
   
   return LaunchDescription([
      declare_use_sim_time,
      
      start_joy_node,
      start_teleop_joy,
      # start_twist_stamper
   ])