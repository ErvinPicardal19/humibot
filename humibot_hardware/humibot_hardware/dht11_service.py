import socket
import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
import serial
import yaml
import os
import socketio

from humibot_interfaces.srv import Humidities as Humidities_SRV
from humibot_interfaces.msg import Humidities as Humidities_MSG

class Dht11_Service(Node):
   
   def __init__(self):
      super().__init__("Dht11_Service")
      
      self.humidities = {"Room_A_Humidity": 0, "Room_B_Humidity": 0}

      self.service_ = self.create_service(
         Humidities_SRV,
         f"/{self.get_name()}/get_humidities",
         self.service_callback
      )

      self.sub_ = self.create_subscription(Humidities_MSG, "DHT11_node/update_humidities", self.update_humidities, 10)

   def update_humidities(self, msg: Humidities_MSG):
      self.get_logger().info(f"Updating humidities: Room A[{msg.room_a_humidity}], RoomB[{msg.room_b_humidity}]")
      self.humidities["Room_A_Humidity"] = msg.room_a_humidity
      self.humidities["Room_B_Humidity"] = msg.room_b_humidity
      
   def service_callback(self, request: Humidities_SRV.Request, response: Humidities_SRV.Response):
      room = request.room
      if room:
         response.humidity = self.humidities[room]
         response.success = True
      else:
         response.humidity = 0
         response.success = False
      
      return response

def main(args=None):
   rclpy.init(args=args)

   dht11_service = Dht11_Service() 
   
   rclpy.spin(dht11_service)
   
   dht11_service.destroy_node()
   rclpy.shutdown()

if __name__ == "__main__":
   main()