from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
   
   default_urdf_path = os.path.join(get_package_share_directory("humibot_description"), "urdf/robot.urdf.xacro")
   
   use_sim_time = LaunchConfiguration("use_sim_time")
   declare_use_sim_time = DeclareLaunchArgument(
      name="use_sim_time",
      default_value="True",
      description="Use Gazebo clock if True"
   )
   urdf_path = LaunchConfiguration("robot_description")
   declare_urdf_path = DeclareLaunchArgument(
      name="robot_description",
      default_value=default_urdf_path,
      description="Full file path of the robot URDF description"
   )

   robot_description = Command(["xacro ", urdf_path])
   params={"robot_description": robot_description, "use_sim_time": use_sim_time}
   start_robot_state_publisher = Node(
      package="robot_state_publisher",
      executable="robot_state_publisher",
      output="screen",
      parameters=[params]
   )
   
   return LaunchDescription([
      declare_use_sim_time,
      declare_urdf_path,
      
      start_robot_state_publisher
   ])