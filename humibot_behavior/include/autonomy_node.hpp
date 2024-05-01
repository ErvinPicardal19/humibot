#ifndef AUTONOMY_NODE_H
#define AUTONOMY_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "dht_serial.hpp"

class AutonomyNode : public rclcpp::Node
{
public:
   AutonomyNode();
   void setup();
   void create_behavior_tree();
   void update_behavior_tree();
   
   private:
      BT::Tree tree_;
      rclcpp::TimerBase::SharedPtr timer_;
};


#endif