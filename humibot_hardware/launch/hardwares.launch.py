from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():\
   
   # dehumidifier_node
   start_dehumidifier_node = Node(
      package="humibot_hardware",
      executable="dehumidifier_service_node",
   )
   
   # dht11_node
   start_dht11_node = Node(
      package="humibot_hardware",
      executable="dht11_node",
   )
   
   # dht11_service
   start_dht11_service = Node(
      package="humibot_hardware",
      executable="dht11_service",
   )
   
   return LaunchDescription([
      start_dehumidifier_node,
      start_dht11_node,
      start_dht11_service,
   ])