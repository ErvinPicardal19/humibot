#ifndef BT_NAVIGATION_H
#define BT_NAVIGATION_H

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "humibot_interfaces/srv/humidities.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "Timer.hpp"

namespace bt_navigation
{

   class IsRoomHumid : public BT::StatefulActionNode
   {
      private:
         rclcpp::Node::SharedPtr node_ptr_;
         bool service_done_;
         double humidity_thresh = 65.0;
         std::string humidity_port_;
         uint8_t humidity_;

         rclcpp::Client<humibot_interfaces::srv::Humidities>::SharedPtr dht11_client_;
      
      public:
         IsRoomHumid(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node_ptr);

         static BT::PortsList providedPorts();

         BT::NodeStatus onStart() override;
         BT::NodeStatus onRunning() override;
         void onHalted() override {};

         void get_humidity_callback(rclcpp::Client<humibot_interfaces::srv::Humidities>::SharedFuture future);
   };

   // BT::NodeStatus get_humidity_callback(rclcpp::Client<humibot_interfaces::srv::Humidities>::SharedFuture future);

   class AskForHelp : public BT::SyncActionNode
   {
      public:
         AskForHelp(const std::string &name);
         BT::NodeStatus tick() override;
   };
   
   class GoToPose : public BT::StatefulActionNode
   {
      public:
         rclcpp::Node::SharedPtr node_ptr_;
         rclcpp_action::ResultCode nav_result_;

         using NavigateToPose = nav2_msgs::action::NavigateToPose;
         using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

         bool done_flag_;
         rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_;

         GoToPose(const std::string name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node_ptr);

         static BT::PortsList providedPorts();

         BT::NodeStatus onStart() override;
         BT::NodeStatus onRunning() override;
         void onHalted() override {};

         void nav_to_pose_callback(const GoalHandleNav::WrappedResult &result);
   };

   class DehumidifyRoom : public BT::StatefulActionNode
   {
      private:
         rclcpp::Node::SharedPtr node_ptr_;
         bool is_dehumidifier_open;
         bool is_dehumidifier_request_sent;
         // bool is_dehumidifier_request_done;
         bool is_humidity_request_sent;
         bool is_humidity_request_done;
         double humidity_thresh = 65.0;
         uint8_t humidity_;

         rclcpp::Client<humibot_interfaces::srv::Humidities>::SharedPtr dht11_client_;
         rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr dehumidifier_client_;

         std::shared_ptr<humibot_interfaces::srv::Humidities::Request> humidity_request;


      public:

         enum DEHUMIDIFIER_SWITCH
         {
            TURN_OFF = false,
            TURN_ON = true
         };

         DehumidifyRoom(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node_ptr);

         static BT::PortsList providedPorts();

         BT::NodeStatus onStart() override;
         BT::NodeStatus onRunning() override;
         void onHalted() override {};

         void dehumidifier_switch_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future);
         
         void get_humidity_callback(rclcpp::Client<humibot_interfaces::srv::Humidities>::SharedFuture future);
         
         void send_dehumidifier_service_request(bool turn_on);
         void send_dht11_service_request();

   };
}

#endif