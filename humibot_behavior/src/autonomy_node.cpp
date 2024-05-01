#include <iostream>
#include <fstream>
#include <chrono>
#include "autonomy_node.hpp"
#include "bt_navigation.hpp"

namespace chr = std::chrono;

std::string bt_xml_dir = ament_index_cpp::get_package_share_directory("humibot_behavior") + "/bt_xml";

AutonomyNode::AutonomyNode() : Node("autonomy_node")
{
   this->declare_parameter("location_file","none");
   RCLCPP_INFO(get_logger(), "Init done");

}

void AutonomyNode::setup()
{
   // initial BT setup
   RCLCPP_INFO(get_logger(), "Setting up");
   create_behavior_tree();
   RCLCPP_INFO(get_logger(), "BT created");

   // setup serial comms
   // this->dht_room_a = std::make_shared<DhtSerial>("/dev/ttyUSB0", LibSerial::BaudRate::BAUD_9600, shared_from_this());
   // this->dht_room_b = std::make_shared<DhtSerial>("/dev/ttyUSB1", LibSerial::BaudRate::BAUD_9600, shared_from_this());

   chr::milliseconds timer_period = chr::milliseconds(500);

   timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&AutonomyNode::update_behavior_tree, this)
   );
   
   rclcpp::spin(shared_from_this());
   rclcpp::shutdown();
}

void AutonomyNode::create_behavior_tree()
{
   BT::BehaviorTreeFactory factory;

   BT::PortsList portsLists = {BT::InputPort<std::string>("humidity")};


   // factory.registerSimpleCondition("IsRoomHumid", std::bind(&bt_navigation::CheckHumidity::isRoomHumid, &checkHumidity, std::placeholders::_1, shared_from_this()), portsLists);

   factory.registerNodeType<bt_navigation::IsRoomHumid>("IsRoomHumid", shared_from_this());

   factory.registerNodeType<bt_navigation::GoToPose>("GoToPose", shared_from_this());

   factory.registerNodeType<bt_navigation::AskForHelp>("AskForHelp");

   factory.registerNodeType<bt_navigation::DehumidifyRoom>("DehumidifyRoom", shared_from_this());

   RCLCPP_INFO(get_logger(), bt_xml_dir.c_str());

   // std::string xml_models = BT::writeTreeNodesModelXML(factory);
   // std::ofstream outfile;
   // outfile.open("groot2.xml");
   // outfile << xml_models;
   // outfile.close();

   tree_ = factory.createTreeFromFile(bt_xml_dir + "/behavior_tree.xml");
   RCLCPP_INFO(this->get_logger(), "3");
}

void AutonomyNode::update_behavior_tree()
{
   tree_.sleep(std::chrono::milliseconds(10));

   // RCLCPP_INFO(this->get_logger(), "--- ticking\n");
   BT::NodeStatus tree_status = tree_.tickOnce();
   
   if (tree_status == BT::NodeStatus::RUNNING)
   {
      // RCLCPP_INFO(this->get_logger(), "--- status: RUNNING\n");
      return;
   }
   else if (tree_status == BT::NodeStatus::SUCCESS)
   {
      RCLCPP_INFO(this->get_logger(), "--- status: SUCCESS\n");
      // timer_->cancel();
   }
   else if (tree_status == BT::NodeStatus::FAILURE)
   {
      RCLCPP_INFO(this->get_logger(), "--- status: FAILED\n");
      timer_->cancel();
   }
}

int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);

   std::shared_ptr<AutonomyNode> node = std::make_shared<AutonomyNode>();

   node->setup();

}