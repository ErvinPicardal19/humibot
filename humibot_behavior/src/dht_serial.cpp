// #include "dht_serial.hpp"
// #include <iostream>

// DhtSerial::DhtSerial(const std::string &port_name, const LibSerial::BaudRate baud_rate, rclcpp::Node::SharedPtr node_ptr)
// {
//    this->node_ptr_ = node_ptr;
//    this->port_name = port_name;
//    // setup SerialPort
//    serial_port = LS::SerialPort(this->port_name);
//    serial_port.SetBaudRate(baud_rate);
//    serial_port.SetCharacterSize(this->message_bytes);
//    this->timeout_ms = 0;
// }


// void DhtSerial::read_port()
// {
//    if(!this->serial_port.IsOpen())
//    {
//       RCLCPP_ERROR(this->node_ptr_->get_logger(), "Cannot open port \033[33m[%s]\033[0m", std::string(this->port_name).c_str());
      
//       return;
//    }

//    // while(!this->serial_port.IsDataAvailable()){}

//    this->serial_port.ReadLine(this->input, '\n', this->timeout_ms);

//    this->humidity = std::stod(this->input) + 0.0;
// }

// DhtSerial::~DhtSerial()
// {
//    RCLCPP_INFO(this->node_ptr_->get_logger(), "Closing port \033[33m[%s]\033[0m...", std::string(this->port_name).c_str());
//    this->serial_port.Close();
// }