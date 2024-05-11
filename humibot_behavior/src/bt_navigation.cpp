#include "bt_navigation.hpp"
#include <string>
#include "yaml-cpp/yaml.h"
#include <vector>
#include <thread>

bt_navigation::AskForHelp::AskForHelp(const std::string &name) : BT::SyncActionNode(name, {}) {}

BT::NodeStatus bt_navigation::AskForHelp::tick()
{
   std::cout << "Asking for help...";
   return BT::NodeStatus::SUCCESS;
}

bt_navigation::IsRoomHumid::IsRoomHumid(
   const std::string &name, 
   const BT::NodeConfig &config,
   rclcpp::Node::SharedPtr node_ptr
) : BT::StatefulActionNode(name, config)
{
   this->node_ptr_ = node_ptr;
   this->dht11_client_ = this->node_ptr_->create_client<humibot_interfaces::srv::Humidities>("/Dht11_Service/get_humidities");
}

BT::PortsList bt_navigation::IsRoomHumid::providedPorts()
{
   return {BT::InputPort<std::string>("room")};
}

BT::NodeStatus bt_navigation::IsRoomHumid::onStart()
{
   if (!this->node_ptr_)
   {
      RCLCPP_ERROR(this->node_ptr_->get_logger(), "ROS2 node not registered via init() method");
      return BT::NodeStatus::FAILURE;
   }
   
   BT::Expected<std::string> room = this->getInput<std::string>("room");

   if(!room)
   {
      throw BT::RuntimeError("missing required input [humidity]");
   }

   auto request = std::make_shared<humibot_interfaces::srv::Humidities::Request>();

   request->room = room.value();

   while (!this->dht11_client_->wait_for_service(std::chrono::milliseconds(1000)))
   {
      if(!rclcpp::ok())
      {
         RCLCPP_ERROR(
            this->node_ptr_->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
         return BT::NodeStatus::SUCCESS;
      }
      RCLCPP_INFO(this->node_ptr_->get_logger(), "Service Unavailable. Waiting for Service...");
   }

   RCLCPP_INFO(this->node_ptr_->get_logger(), "...");
   RCLCPP_INFO(this->node_ptr_->get_logger(), "--- status: ROOM \33[33m[%s]\33[0m\n", this->name().c_str());
   RCLCPP_INFO(this->node_ptr_->get_logger(), "Sending request to fetch humidity data");

   this->service_done_ = false;

   this->dht11_client_->async_send_request(
      request,
      std::bind(&bt_navigation::IsRoomHumid::get_humidity_callback, this, std::placeholders::_1)
   );

   return BT::NodeStatus::RUNNING;
}

BT::NodeStatus bt_navigation::IsRoomHumid::onRunning()
{
   if (this->service_done_)
   {
      if (this->humidity_ >= this->humidity_thresh)
      {
         RCLCPP_INFO(this->node_ptr_->get_logger(), "\033[33;1;4mHumidity exceeded {%.2f}%% threshold\033[0m", this->humidity_thresh);
         RCLCPP_INFO(this->node_ptr_->get_logger(), "Navigating to \033[33m%s\033[0m\n", this->name().c_str());
         return BT::NodeStatus::FAILURE;
      }
      else if (this->humidity_ <= 0)
      {
         RCLCPP_WARN(this->node_ptr_->get_logger(), "\33[33mSensor at room [%s] is not working\33[0m\n", this->name().c_str());
         return BT::NodeStatus::SUCCESS;
      }
      else
      {
         RCLCPP_INFO(this->node_ptr_->get_logger(), "Humidity is normal at \033[33m%s\033[0m\n", this->name().c_str());
         return BT::NodeStatus::SUCCESS;
      }
   }
   else
   {
      return BT::NodeStatus::RUNNING;
   }

}

void bt_navigation::IsRoomHumid::get_humidity_callback(rclcpp::Client<humibot_interfaces::srv::Humidities>::SharedFuture future)
{

   auto status = future.wait_for(std::chrono::milliseconds(1000));

   if(status == std::future_status::ready)
   {
      RCLCPP_INFO(this->node_ptr_->get_logger(), "\33[33m[%d%%]\33[0m Humidity at \33[33m%s\33[0m", future.get()->humidity, this->name().c_str());

      this->humidity_ = future.get()->humidity; 
      this->service_done_ = true;
   }
   else
   {
      RCLCPP_INFO(this->node_ptr_->get_logger(), "Service In-Progress...");
   }
}

bt_navigation::GoToPose::GoToPose(const std::string name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node_ptr) : BT::StatefulActionNode(name, config), node_ptr_{node_ptr}
{
};

BT::PortsList bt_navigation::GoToPose::providedPorts()
{
   return {BT::InputPort<std::string>("loc")};
}

BT::NodeStatus bt_navigation::GoToPose::onStart()
{

   // Validate if node exists
   if (!node_ptr_)
   {
      RCLCPP_ERROR(this->node_ptr_->get_logger(), "ROS2 node not registered via init() method");
      return BT::NodeStatus::FAILURE;
   }

   // Read YAML file
   BT::Expected<std::string> loc = this->getInput<std::string>("loc");
   const std::string location_file = node_ptr_->get_parameter("location_file").as_string();

   YAML::Node locations = YAML::LoadFile(location_file);
   
   std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

   // setup the action client
   auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
   // send_goal_options.feedback_callback = std::bind(&GoToPose::nav_to_pose_feedback, this, _1, _2);
   send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);
   action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
   
   auto goal_msg = NavigateToPose::Goal();
   goal_msg.pose.header.frame_id = "map";
   goal_msg.pose.pose.position.x = pose[0];
   goal_msg.pose.pose.position.y = pose[1];
   
   tf2::Quaternion q;
   q.setRPY(0, 0, pose[2]);
   q.normalize();
   goal_msg.pose.pose.orientation = tf2::toMsg(q);

   // send the navigation action goal
   done_flag_ = false;
   action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
   RCLCPP_INFO(this->node_ptr_->get_logger(), "\033[33m[%s]\033[0m Sent goal message", std::string(this->name()).data());

   return BT::NodeStatus::RUNNING;
}

BT::NodeStatus bt_navigation::GoToPose::onRunning()
{
   if(this->done_flag_)
   {
      if(nav_result_ == rclcpp_action::ResultCode::SUCCEEDED)
      {
         RCLCPP_INFO(this->node_ptr_->get_logger(), "\033[33m[%s]\033[0m Goal reached\n", std::string(this->name()).data());
         return BT::NodeStatus::SUCCESS;
      }
      else
      {
         RCLCPP_ERROR(this->node_ptr_->get_logger(), "\033[33m[%s]\033[0m Failed to reach goal\n", std::string(this->name()).data());
            return BT::NodeStatus::FAILURE;  
      }
   }
   else
   {
      return BT::NodeStatus::RUNNING;
   }
}

void bt_navigation::GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
   // If there is a result, we consider navigation completed and save the
    // result code to be checked in the `onRunning()` method.
   if (result.result)
   {
      done_flag_ = true;
      nav_result_ = result.code;
   }
}

bt_navigation::DehumidifyRoom::DehumidifyRoom(
   const std::string &name, 
   const BT::NodeConfig &config,
   rclcpp::Node::SharedPtr node_ptr
) : BT::StatefulActionNode(name, config)
{
   this->node_ptr_ = node_ptr;
   this->dehumidifier_client_ = this->node_ptr_->create_client<std_srvs::srv::SetBool>("/dehumidifier_switch");
   this->dht11_client_ = this->node_ptr_->create_client<humibot_interfaces::srv::Humidities>("/Dht11_Service/get_humidities");

}

BT::PortsList bt_navigation::DehumidifyRoom::providedPorts()
{
   return {BT::InputPort<std::string>("room")};
}

BT::NodeStatus bt_navigation::DehumidifyRoom::onStart()
{

   // Validate if node exists
   if (!this->node_ptr_)
   {
      RCLCPP_ERROR(this->node_ptr_->get_logger(), "ROS2 node not registered via init() method");
      return BT::NodeStatus::FAILURE;
   }

   while (
      !this->dht11_client_->wait_for_service(std::chrono::milliseconds(1000)) && !this->dehumidifier_client_->wait_for_service(std::chrono::milliseconds(1000))
   )
   {
      if(rclcpp::ok())
      {
         RCLCPP_ERROR(
            this->node_ptr_->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
         return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(this->node_ptr_->get_logger(), "Service Unavailable. Waiting for Service...");
   }

   RCLCPP_INFO(this->node_ptr_->get_logger(), "Sending request to start dehumidifier");

   this->is_dehumidifier_open = false;
   this->is_humidity_request_done = false;
   this->is_humidity_request_sent = false;
   this->is_dehumidifier_request_sent = false;
   // this->is_dehumidifier_request_done = false;

   // this->send_dehumidifier_service_request(bt_navigation::DehumidifyRoom::DEHUMIDIFIER_SWITCH::TURN_ON);

   return BT::NodeStatus::RUNNING;
}

BT::NodeStatus bt_navigation::DehumidifyRoom::onRunning()
{
   if (!this->is_humidity_request_sent)
   {
      this->send_dht11_service_request();
      this->is_humidity_request_sent = true;
   }

   if (!this->is_dehumidifier_request_sent && !this->is_dehumidifier_open)
   {
      this->send_dehumidifier_service_request(bt_navigation::DehumidifyRoom::DEHUMIDIFIER_SWITCH::TURN_ON);
      this->is_dehumidifier_request_sent = true;
   }

   if(this->is_dehumidifier_open)
   {
      if (this->is_humidity_request_done)
      {
         if (this->humidity_ < this->humidity_thresh)
         {

            RCLCPP_INFO(node_ptr_->get_logger(), "Reached normal humidity threshold");
            RCLCPP_INFO(node_ptr_->get_logger(), "Turning off dehumidifier.");

            this->send_dehumidifier_service_request(bt_navigation::DehumidifyRoom::DEHUMIDIFIER_SWITCH::TURN_OFF);

            // this->is_dehumidifier_open = false;

            return BT::NodeStatus::SUCCESS;
         }
         else if (this->humidity_ <= 0)
         {
            RCLCPP_WARN(this->node_ptr_->get_logger(), "\33[33mSensor at [%s] is not working\33[0m\n", this->name().c_str());

            this->send_dehumidifier_service_request(bt_navigation::DehumidifyRoom::DEHUMIDIFIER_SWITCH::TURN_OFF);

            return BT::NodeStatus::SUCCESS;
         }

         is_humidity_request_done = false;
      }
      else
      {
         RCLCPP_INFO(this->node_ptr_->get_logger(), "Waiting for humidity data...");
      }
   }
   else
   {
      RCLCPP_INFO(this->node_ptr_->get_logger(), "Waiting for dehumidifier to turn on...");
   }

   return BT::NodeStatus::RUNNING;
}

void bt_navigation::DehumidifyRoom::dehumidifier_switch_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
{
   auto status = future.wait_for(std::chrono::milliseconds(1000));

   if (status == std::future_status::ready)
   {
      RCLCPP_INFO(this->node_ptr_->get_logger(), "Result: success: %i, message: %s", future.get()->success, future.get()->message.c_str());
      this->is_dehumidifier_open = future.get()->success;
      this->is_dehumidifier_request_sent = false;
   }
   else
   {
      RCLCPP_INFO(this->node_ptr_->get_logger(), "Service In-Progress...");
   }
}

void bt_navigation::DehumidifyRoom::get_humidity_callback(rclcpp::Client<humibot_interfaces::srv::Humidities>::SharedFuture future)
{

   auto status = future.wait_for(std::chrono::milliseconds(1000));

   if(status == std::future_status::ready)
   {
      RCLCPP_INFO(this->node_ptr_->get_logger(), "\33[33m[%d%%]\33[0m Humidity at \33[33m%s\33[0m", future.get()->humidity, this->name().c_str());

      this->humidity_ = future.get()->humidity; 
      this->is_humidity_request_sent = false;
      this->is_humidity_request_done = true;
   }
   else
   {
      RCLCPP_INFO(this->node_ptr_->get_logger(), "Service In-Progress...");
   }
}

void bt_navigation::DehumidifyRoom::send_dehumidifier_service_request(bool dehumidifier_switch)
{
   // Send service request
   auto dehumidifier_request = std::make_shared<std_srvs::srv::SetBool::Request>();

   dehumidifier_request->data = dehumidifier_switch;

   // RCLCPP_INFO(this->node_ptr_->get_logger(), "Sending request to dehumidifier service");

   this->dehumidifier_client_->async_send_request(
      dehumidifier_request,
      std::bind(&bt_navigation::DehumidifyRoom::dehumidifier_switch_callback, this, std::placeholders::_1)
   );

   return;
}

void bt_navigation::DehumidifyRoom::send_dht11_service_request()
{

   BT::Expected<std::string> room = this->getInput<std::string>("room");

   if(!room)
   {
      throw BT::RuntimeError("missing required input [room]");
   }

   // Creating service request
   humidity_request = std::make_shared<humibot_interfaces::srv::Humidities::Request>();

   humidity_request->room = room.value();

   // RCLCPP_INFO(this->node_ptr_->get_logger(), "Sending request to dht11 service");

   // Send service request
   this->dht11_client_->async_send_request(
      humidity_request,
      std::bind(&bt_navigation::DehumidifyRoom::get_humidity_callback, this, std::placeholders::_1)
   );

   return;
}