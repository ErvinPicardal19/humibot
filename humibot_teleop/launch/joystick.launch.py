from launch import LaunchContext, LaunchDescription, condition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import LaunchConfigurationEquals

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
   
   
   # controller_brand = LaunchConfiguration("controller_brand")
   declare_controller_type = DeclareLaunchArgument(
      name="controller_type",
      default_value="dobe",
      description="Declare what type/brand is the controller"
   )

   
   start_joy_node = Node(
      package="joy",
      executable="joy_node",
      parameters=[joy_params, {"use_sim_time": use_sim_time}]
   )

   start_teleop_joy_ps4_controller = Node(
      condition=LaunchConfigurationEquals("controller_type", "ps4"),
      package="teleop_twist_joy",
      executable="teleop_node",
      name="ps4_teleop_node",
      parameters=[joy_params, {"use_sim_time": use_sim_time}],
      remappings=[
         ("/cmd_vel", "/cmd_vel_joy")
      ]
   )
   
   start_teleop_joy_dobe_controller = Node(
      condition=LaunchConfigurationEquals("controller_type", "dobe"),
      package="teleop_twist_joy",
      executable="teleop_node",
      name="dobe_teleop_node",
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
      declare_controller_type,
      
      start_joy_node,
      start_teleop_joy_ps4_controller,
      start_teleop_joy_dobe_controller,
      # start_twist_stamper
   ])