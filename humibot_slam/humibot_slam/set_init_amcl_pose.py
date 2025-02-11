import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import transforms3d

class InitAmclPosePublisher(Node):
   def __init__(self):
      super().__init__("init_amcl_pose_publisher")
      
      self.declare_parameter("x", value=0.0)
      self.declare_parameter("y", value=0.0)
      self.declare_parameter("theta", value=0.0)
      self.declare_parameter("cov", value=0.5**2)
      
      self.pub_ = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
      
      while(self.pub_.get_subscription_count() == 0):
         self.get_logger().info("Waiting for AMCL Initial Pose subscriber")
         time.sleep(1.0)
   
   def publish_init_pose(self):
      x = self.get_parameter("x").value
      y = self.get_parameter("y").value
      theta = self.get_parameter("theta").value
      cov = self.get_parameter("cov").value
      
      msg = PoseWithCovarianceStamped()
      msg.header.frame_id = "map"
      msg.pose.pose.position.x = x
      msg.pose.pose.position.y = y
      
      quat = transforms3d.euler.euler2quat(0, 0, theta)
      msg.pose.pose.orientation.w = quat[0]
      msg.pose.pose.orientation.x = quat[1]
      msg.pose.pose.orientation.y = quat[2]
      msg.pose.pose.orientation.z = quat[3]
      
      msg.pose.covariance = [
         cov, 0.0, 0.0, 0.0, 0.0, 0.0, # Pos X
         0.0, cov, 0.0, 0.0, 0.0, 0.0, # Pos Y
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # Pos Z
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # Rot X
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # Rot Y
         0.0, 0.0, 0.0, 0.0, 0.0, cov  # Rot Z
      ]
      
      self.pub_.publish(msg)
      

def main(args=None):
   rclpy.init(args=args)
   
   init_amcl_pose_publisher = InitAmclPosePublisher()
   
   init_amcl_pose_publisher.publish_init_pose()
   
   rclpy.spin(init_amcl_pose_publisher)
   
   init_amcl_pose_publisher.destroy_node()
   rclpy.shutdown()

if __name__ == "__main__":
   main()