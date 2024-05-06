from os import stat
import rclpy
import rclpy.logging
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Bool

import signal
import sys
import atexit

import RPi.GPIO as GPIO

class DehumidifierServiceNode(Node):
   def __init__(self):
      super().__init__("dehumidifier_service_node")
      
      self.dehumidifier_status = False
      self.dehumidifier_pin = 17
      self.service_ = self.create_service(
         SetBool, 
         "/dehumidifier_switch",
         self.switch
      )
      self.pub_ = self.create_publisher(Bool, "/dehumidifier_status", 10)
      
      
      period_sec = 1;
      self.timer_ = self.create_timer(period_sec, self.publish_status)
      
      
      GPIO.setmode(GPIO.BCM)
      GPIO.setup(self.dehumidifier_pin, GPIO.OUT)

      atexit.register(self.keyboard_interrupt_handler)
      self.get_logger().info(f"{self.get_name()} started")
      
   
   def publish_status(self):
      status_msg = Bool()
      status_msg.data = self.dehumidifier_status
      
      self.pub_.publish(status_msg)
   
   def keyboard_interrupt_handler(self):
      GPIO.cleanup()
   
   def switch(self, request: SetBool.Request, response: SetBool.Response):
      self.get_logger().info(f"Received data {request.data}")
      
      try:
         if request.data:
            self.get_logger().info(f"Turning on dehumidifier")
            response.message = "Dehumidifier turned on"
            
            self.dehumidifier_status = True
            GPIO.output(self.dehumidifier_pin, GPIO.HIGH)
            
         else:
            self.get_logger().info(f"Turning off dehumidifier")
            response.message = "Dehumidifier turned off"
            
            self.dehumidifier_status = False
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
   
   rclpy.shutdown()

if __name__ == "__main__":
   main()

      