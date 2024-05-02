// #ifndef DHT_SENSOR_H
// #define DHT_SENSOR_H

// #include<string>
// #include<libserial/SerialPort.h>
// #include<libserial/SerialStream.h>
// #include "rclcpp/rclcpp.hpp"
// // #include "rclcpp_action/rclcpp_action.hpp"

// namespace LS = LibSerial;

// class DhtSerial
// {
//    private:
//       std::string input = "";
//       size_t timeout_ms = 0;
//       LS::CharacterSize message_bytes = LS::CharacterSize::CHAR_SIZE_8;

      
//    public:
//       double humidity;
//       rclcpp::Node::SharedPtr node_ptr_;
//       std::string port_name;
//       LS::SerialPort serial_port;

//       DhtSerial(const std::string &port_name, const LS::BaudRate baud_rate, rclcpp::Node::SharedPtr node_ptr);
//       // DhtSerial(const LibSerial::BaudRate baud_rate, rclcpp::Node::SharedPtr node_ptr);

//       ~DhtSerial();

//       void read_port();
// };

// #endif