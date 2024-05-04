import rclpy
import rclpy.logging
from rclpy.node import Node
from std_srvs.srv import SetBool

import signal
import sys
import atexit

import RPi.GPIO as GPIO

class DehumidifierServiceNode(Node):
   def __init__(self):
      super().__init__("dehumidifier_service_node")
      self.dehumidifier_pin = 17
      self.service_ = self.create_service(
         SetBool, 
         "/dehumidifier_switch",
         self.switch
      )
      GPIO.setmode(GPIO.BCM)
      GPIO.setup(self.dehumidifier_pin, GPIO.OUT)
      
      atexit.register(self.__del__)
   
   def __del__(self):
      self.get_logger().info("Cleaning up GPIO")
      print("Cleaning up GPIO")
      GPIO.cleanup()
      
   def switch(self, request: SetBool.Request, response: SetBool.Response):
      self.get_logger().info(f"Received data {request.data}")
      
      try:
         if request.data:
            self.get_logger().info(f"Turning on dehumidifier")
            response.message = "Dehumidifier turned on"
            
            GPIO.output(self.dehumidifier_pin, GPIO.HIGH)
            
         else:
            self.get_logger().info(f"Turning off dehumidifier")
            response.message = "Dehumidifier turned off"
            
            GPIO.output(self.dehumidifier_pin, GPIO.LOW)
            
         response.success = True
      except:
         response.success = False
         response.message = "Cannot access dehumidifier";
      
      return response
      
def main(args=None):
   rclpy.init(args=args)
   
   dehumidifier_service_node = DehumidifierServiceNode()

   rclpy.spin(dehumidifier_service_node)
   
   dehumidifier_service_node.destroy_node()
   
   GPIO.cleanup()
   rclpy.shutdown()

if __name__ == "__main__":
   main()