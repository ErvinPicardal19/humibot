import time
import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
import serial
import yaml
import os

import time

from humibot_interfaces.msg import Humidities

class SerialPort:
   def __init__(self, port_name=None, baudrate=9600, bytesize=8, timeout=0, stopbits=serial.STOPBITS_ONE) -> None:
      
      self.port_name = port_name
      self.baudrate = baudrate
      self.bytesize = bytesize
      self.timeout = timeout
      self.stopbits = stopbits
      
      self.port = serial.Serial(port=self.port_name, baudrate=self.baudrate, bytesize=self.bytesize, timeout=self.timeout, stopbits=self.stopbits)
      
      self.is_open = self.port.is_open
      
   def __del__(self) -> None:
      self.port.close()
      self.port.is_open = False
   
   def read(self):
      dt_ms = time.time() * 1000.0
      while(1):
         if (self.port.in_waiting > 0):   
            return self.port.readline().decode('Ascii')
         
         # timeout if 2s passed and return 0
         if (dt_ms - (time.time()*1000) > 2000):
            return "0"

class Dht11_Node(Node):
   def __init__(self):
      super().__init__("DHT11_node") # type: ignore
      # self.humidities = {"humidity1": 0, "humidity2": 0}
      
      # self.humidities_yaml = os.path.join(get_package_share_directory("humibot_behavior"), "config/humidities.yaml");

      self.serial_port1 = SerialPort()
      self.serial_port2 = SerialPort()
      
      # self.service_ = self.create_service(
      #    Humidities,
      #    f"{self.get_name()}/get_humidities",
      #    self.service_callback
      # )
      
      self.pub_ = self.create_publisher(Humidities, f"{self.get_name()}/update_humidities", 10)
      
      period_sec = 1;
      self.timer_ = self.create_timer(period_sec, self.update_humidities)
   
   def update_humidities(self):
      
      humidities = Humidities()
      
      # Open ports
      self.serial_port1.port.port = "/dev/ttyUSB0"
      self.serial_port2.port.port = "/dev/ttyUSB1"
      if(not self.serial_port1.is_open or not self.serial_port2.is_open):
         try:
            self.serial_port1.port.open()
            self.serial_port1.is_open = True
            
            if(self.serial_port1.is_open):
               port1_output = self.serial_port1.read()
         
            if(port1_output):
               # self.humidities["humidity1"] = int(float(port1_output))
               humidities.room_a_humidity = int(float(port1_output))
         except:
            self.get_logger().warn(f"Cannot access port [{self.serial_port1.port.port}]")
            # self.serial_port1.is_open = False
         try:
            self.serial_port2.port.open()
            self.serial_port2.is_open = True
            
            if(self.serial_port2.is_open):
               port2_output = self.serial_port2.read()
         
            if(port2_output):
               # self.humidities["humidity2"] = int(float(port2_output))
               humidities.room_b_humidity = int(float(port2_output))
         except:
            self.get_logger().warn(f"Cannot access port [{self.serial_port2.port.port}]")
            # self.serial_port2.is_open = False
         
      self.get_logger().info(f"Room A Humidity: {humidities.room_a_humidity}")
      self.get_logger().info(f"Room B Humidity: {humidities.room_b_humidity}")
      
      # with open(self.humidities_yaml, 'w') as file:
      #    self.get_logger().info("updating humidity file...")
      #    documents = yaml.dump(humidities, file)
      #    self.get_logger().info("file updated")
         
      #    file.close()

      self.pub_.publish(humidities)

      self.serial_port1.port.close()
      self.serial_port1.is_open = False
      self.serial_port2.port.close()
      self.serial_port1.is_open = False
      
      return
   
   # def service_callback(self, request: Humidities.Request, response: Humidities.Response):
   #    room = request.room
   #    if room:
   #       response.humidity = self.humidities[room]
   #       response.success = True
   #    else:
   #       response.humidity = 0
   #       response.success = False
      
   #    return response

def main(args=None):
   rclpy.init(args=args)
   
   dht11_node = Dht11_Node()
   
   rclpy.spin(dht11_node)
   
   dht11_node.destroy_node()
   rclpy.shutdown()

if __name__ == "__main__":
   main()