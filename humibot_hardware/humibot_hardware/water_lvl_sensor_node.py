import time
import rclpy
from rclpy.node import Node
import serial
import time
import atexit

from std_msgs.msg import Int16

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
      # dt_ms = time.time() * 1000.0
      while(1):
         data = self.port.readline().decode().strip()
         if (data):
            # water_lvl = self.port.readline().decode('utf-8')
            # print(data)
            return data

class WaterLvlNode(Node):
   def __init__(self):
      super().__init__("water_lvl_node") # type: ignore
      
      self.declare_parameter('port', value='/dev/WaterSensor')

      self.serial_port = SerialPort()
      
      self.pub_ = self.create_publisher(Int16, f"{self.get_name()}/update_water_lvl", 10)
      
      period_sec = 1;
      self.timer_ = self.create_timer(period_sec, self.update_humidities)
   
      self.serial_port.port.port = self.get_parameter('port').get_parameter_value().string_value
         
      atexit.register(self.cleanup)
      self.get_logger().info(f"{self.get_name()} started")
      
   def cleanup(self):
      self.serial_port.port.close()
      self.serial_port.is_open = False
   
   def update_humidities(self):
      
      water_lvl = Int16()
      water_lvl.data = 0
      
      try:
         if not self.serial_port.port.is_open:
            self.serial_port.port.open()
      except:
         self.get_logger().warn(f"Cannot access port [{self.serial_port.port.port}]")
      
      # Open ports
      # self.serial_port.port.port = "/dev/ttyUSB1"
      if(self.serial_port.port.is_open):

         port_output = self.serial_port.read()

         if(port_output):
            # self.humidities["humidity1"] = int(float(port_output))
            water_lvl.data = int(float(port_output))
         
      # self.get_logger().info(f"Water Level: {water_lvl.data} mL")

      self.pub_.publish(water_lvl)

      return

def main(args=None):
   rclpy.init(args=args)
   
   waterLvlNode = WaterLvlNode()
   
   rclpy.spin(waterLvlNode)
   
   waterLvlNode.destroy_node()
   rclpy.shutdown()

if __name__ == "__main__":
   main()