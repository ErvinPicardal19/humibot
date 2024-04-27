from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro

def generate_launch_description():
   
   humibot_description_pkg = get_package_share_directory("humibot_description")
   
   rviz_config_file = LaunchConfiguration("rviz_config")
   declare_rviz_config_file = DeclareLaunchArgument(
      name="rviz_config",
      default_value=os.path.join(humibot_description_pkg, "rviz/real.rviz"),
      description="Full file path for rviz2 config file"
   )
   
   start_robot_state_publisher = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(humibot_description_pkg, "launch/rsp.launch.py")),
      launch_arguments={"use_sim_time": "True"}.items()
   )
   
   start_rviz = Node(
      package="rviz2",
      executable="rviz2",
      arguments=[
         "-d", rviz_config_file
      ],
      parameters=[
         {"use_sim_time": True}
      ]
   )
   
   start_joint_state_publisher = Node(
      package="joint_state_publisher",
      executable="joint_state_publisher",
      output="screen"
   )
   
   return LaunchDescription([
      declare_rviz_config_file,
      
      start_robot_state_publisher,
      start_rviz,
      start_joint_state_publisher
   ])