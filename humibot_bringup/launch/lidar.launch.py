from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
   
   topic_name = LaunchConfiguration("topic_name")
   serial_port = LaunchConfiguration("serial_port")
   lidar_frame = LaunchConfiguration("lidar_frame")
   
   declare_topic_name = DeclareLaunchArgument(
      name="topic_name",
      default_value="/scan",
      description="topic name for publishing laserscan"
   )
   declare_serial_port = DeclareLaunchArgument(
      name="serial_port",
      default_value="/dev/LD06",
      description="device serial port for LD06 lidar"
   )
   declare_lidar_frame = DeclareLaunchArgument(
      name="lidar_frame",
      default_value="lidar_link",
      description="Lidar Frame ID"
   )
   
   
   start_ldlidar = Node(
      package="ldlidar",
      executable="ldlidar",
      parameters=[
         {
            "topic_name": topic_name,
            "serial_port": serial_port,
            "lidar_frame": lidar_frame
         }
      ]
   )
   
   return LaunchDescription([
      declare_topic_name,
      declare_serial_port,
      declare_lidar_frame,
      
      start_ldlidar
   ])