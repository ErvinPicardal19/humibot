import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool

class DehumidifierServiceNode(Node):
   def __init__(self):
      super().__init__("dehumidifier_service_node")
   
      self.service_ = self.create_service(
         SetBool, 
         "/dehumidifier_switch",
         self.switch
      )
      
   def switch(self, request: SetBool.Request, response: SetBool.Response):
      
      self.get_logger().info(f"Received data {request.data}")
      
      try:
         if request.data:
            self.get_logger().info(f"Turning on dehumidifier")
            response.message = "Dehumidifier turned on"
         else:
            self.get_logger().info(f"Turning off dehumidifier")
            response.message = "Dehumidifier turned off"
            
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