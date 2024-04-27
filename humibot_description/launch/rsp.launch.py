from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os
import launch_ros.parameter_descriptions

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
   use_ros2_control = LaunchConfiguration("use_ros2_control")
   declare_use_ros2_control = DeclareLaunchArgument(
      name="use_ros2_control",
      default_value="True",
      description="Use ros2_control if True"
   )
   
   robot_description_raw = Command(["xacro ", urdf_path, " use_ros2_control:=", use_ros2_control, " use_sim_mode:=", use_sim_time])
   
   robot_description = launch_ros.parameter_descriptions.ParameterValue(robot_description_raw, value_type=str)
   
   
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
      declare_use_ros2_control,
      
      start_robot_state_publisher
   ])