import rclpy
from rclpy.node import Node
import time

import socketio

from humibot_interfaces.msg import Humidities as Humidities_MSG
from std_msgs.msg import Int16

class WSServiceNode(Node):
   def __init__(self):
      super().__init__("websocket_service_node")
      
      self.humidities = {"Room_A_Humidity": 0, "Room_B_Humidity": 0}
      self.water_lvl = 0.0
      
      self.declare_parameter('url', value='http://192.168.1.10:5000')
      self.socket_url = self.get_parameter('url').get_parameter_value().string_value


      self.sub_1 = self.create_subscription(Humidities_MSG, "DHT11_node/update_humidities", self.get_humidity_callback, 10)
      self.sub_2 = self.create_subscription(Int16, "water_lvl_node/update_water_lvl", self.get_water_lvl_callback, 10)

      # while(self.count_publishers('water_lvl_node/update_water_lvl') <= 0):
      #    self.get_logger().info("Waiting for DHT11_node and water_lvl_node...")
      #    time.sleep(1)

      self.timer_ = self.create_timer(1, self.check_interrupt)
      try:
         self.sio = socketio.Client()
         self.sio.on('connect', self.on_connect)
         self.sio.on('disconnect', self.on_disconnect)
         
         self.sio.connect(self.socket_url, transports=['websocket'])
         
      except:
         self.get_logger().error('Cannot reach websocket server')
   
   def check_interrupt(self):
      pass

   def on_connect(self):
      self.sio.emit('robot_connect', "")
      self.get_logger().info('Connected to Websocket Server')

   def on_disconnect(self):
      self.get_logger().info('Websocket Server Disconnected')

   def get_humidity_callback(self, msg: Humidities_MSG):
      self.humidities['Room_A_Humidity'] = msg.room_a_humidity
      self.humidities['Room_B_Humidity'] = msg.room_b_humidity
      
      try:
         self.sio.emit('humidities', self.humidities)
      except:
         self.get_logger().error('Websocket not connected...')

   def get_water_lvl_callback(self, msg: Int16):
      self.water_lvl = msg.data
      
      try:
         self.sio.emit('water_lvl_update', self.water_lvl)
      except:
         self.get_logger().error('Websocket not connected...')

def main(args=None):
   rclpy.init(args=args)
   
   wsServiceNode = WSServiceNode()
   rclpy.spin(wsServiceNode)
   
   wsServiceNode.destroy_node()
   rclpy.shutdown()

if __name__ == "__main__":
   main()