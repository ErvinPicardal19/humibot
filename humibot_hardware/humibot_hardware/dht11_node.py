import time
import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
import serial
import atexit

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
         data = self.port.readline().decode().strip()
         if (data):   
            
            return data
         
         # timeout if 2s passed and return 0
         if (dt_ms - (time.time()*1000) > 2000):
            return "0"

class Dht11_Node(Node):
   def __init__(self):
      super().__init__("DHT11_node") # type: ignore

      self.serial_port1 = SerialPort()
      self.serial_port2 = SerialPort()

      
      self.pub_ = self.create_publisher(Humidities, f"{self.get_name()}/update_humidities", 10)
      
      period_sec = 1;
      self.timer_ = self.create_timer(period_sec, self.update_humidities)
      
      
      self.serial_port1.port.port = "/dev/ttyACM0"
      self.serial_port2.port.port = "/dev/ttyUSB0"
      
      try:
         self.serial_port1.port.open()
         self.serial_port1.is_open = True
      except:
         self.get_logger().warn(f"Cannot access port [{self.serial_port1.port.port}]")
      try:
         self.serial_port2.port.open()
         self.serial_port2.is_open = True
      except:
         self.get_logger().warn(f"Cannot access port [{self.serial_port2.port.port}]")
         
      atexit.register(self.cleanup)
      self.get_logger().info(f"{self.get_name()} started")
      
   def cleanup(self):
      self.serial_port1.port.close()
      self.serial_port1.is_open = False
      self.serial_port2.port.close()
      self.serial_port1.is_open = False
   
   def update_humidities(self):
      
      humidities = Humidities()
      
      # Open ports
      if(self.serial_port1.is_open and self.serial_port2.is_open):

         port1_output = self.serial_port1.read()
         port2_output = self.serial_port2.read()
   
         if(port1_output):
            # self.humidities["humidity1"] = int(float(port1_output))
            humidities.room_a_humidity = int(float(port1_output))

         if(port2_output):
            # self.humidities["humidity2"] = int(float(port2_output))
            humidities.room_b_humidity = int(float(port2_output))
       
         
      self.get_logger().info(f"Room A Humidity: {humidities.room_a_humidity}")
      self.get_logger().info(f"Room B Humidity: {humidities.room_b_humidity}")


      self.pub_.publish(humidities)

      # self.serial_port1.port.close()
      # self.serial_port1.is_open = False
      # self.serial_port2.port.close()
      # self.serial_port1.is_open = False
      
      return

def main(args=None):
   rclpy.init(args=args)
   
   dht11_node = Dht11_Node()
   
   rclpy.spin(dht11_node)
   
   dht11_node.destroy_node()
   rclpy.shutdown()

if __name__ == "__main__":
   main()