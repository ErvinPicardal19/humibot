from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
   
   humibot_slam_pkg = get_package_share_directory("humibot_slam")
   humibot_description_pkg = get_package_share_directory("humibot_description")
   
   use_sim_time = LaunchConfiguration("use_sim_time")
   declare_use_sim_time = DeclareLaunchArgument(
      name="use_sim_time",
      default_value="False",
      description="Use Gazebo clock if True"
   )

   rviz_config_file = LaunchConfiguration("rviz_config")
   declare_rviz_config_file = DeclareLaunchArgument(
      name="rviz_config",
      default_value=os.path.join(humibot_description_pkg, "rviz/mapping.rviz"),
      description="Rviz2 config file full path"
   )
   
   start_mapping = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
         os.path.join(humibot_slam_pkg, "launch/online_async_launch.py")
      ]),
      launch_arguments={"use_sim_time": use_sim_time}.items()
   )
   
   start_rviz = Node(
      package="rviz2",
      executable="rviz2",
      arguments=[
         "-d", rviz_config_file
      ],
      parameters=[{"use_sim_time": use_sim_time}]
   )
   
   return LaunchDescription([
      declare_use_sim_time,
      declare_rviz_config_file,
      
      start_mapping,
      start_rviz
   ])