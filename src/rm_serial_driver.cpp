// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

// WFY: Debug only
#include <cstdio>


union FloatUnion {
    float floatValue;
    unsigned char byteArray[sizeof(float)];
};

/* Reverse Float Eddian Helper Function */
float reverseFloatEndian(float value) {
    union FloatUnion u;
    u.floatValue = value;

    // Byte Reverse
    unsigned char temp;
    temp = u.byteArray[0];
    u.byteArray[0] = u.byteArray[3];
    u.byteArray[3] = temp;

    temp = u.byteArray[1];
    u.byteArray[1] = u.byteArray[2];
    u.byteArray[2] = temp;

    return u.floatValue;
}

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create Publisher
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  aiming_point_.header.frame_id = "odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // Create Subscription
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  int remain;
  int taken;

  // uint16_t seq_num;
  uint8_t data_len = 0;
  uint8_t cmd_id;
  uint8_t packet[21];
  int pack_len = 0;

  std::vector<uint8_t> header;   // number of bytes before data
  std::vector<uint8_t> data;
  std::vector<uint8_t> tail(2);
  // data.reserve(sizeof(ReceivePacket));
  // data.reserve(sizeof(ReceivePacket)); //

  while (rclcpp::ok()) {
    try {
      header.resize(1);
      remain = serial_driver_->port()->receive(header);  //
      if (header[0] == 'S' && remain > 0){
        remain = serial_driver_->port()->receive(header);     
        if (header[0] == 'T' && remain > 0) {    
          // data.resize(sizeof(ReceivePacket) - 2);
          data.clear();
          header.clear();
          remain = 4;
          while(remain > 0){
            data.resize(remain);
            taken = serial_driver_->port()->receive(data);
            remain -= taken;
            header.insert(header.end(), data.begin(), data.begin() + taken);
          }

          if(header[2]==0){
            RCLCPP_INFO(this->get_logger(),"%d",(int)header.size());
          }
          // seq_num = header[0] + (header[1] << 8);     // Little Endian, First low 8 bytes
          data_len = header[2];
          cmd_id = header[3];
          
          data.clear();
          remain = data_len + 3;
          while(remain > 0){
            tail.resize(remain);
            taken = serial_driver_->port()->receive(tail);
            remain -= taken;
            data.insert(data.end(),tail.begin(),tail.begin()+taken);
          }
          auto tail1 = data.end() - 2;
          auto tail2 = data.end() - 1;
          // RCLCPP_INFO(this->get_logger(), "TAIL: %d, %d", *tail1, *tail2);
         
          packet[0] = 'S';
          packet[1] = 'T';
          for(int i = 0; i < 4; i++){
            packet[i+2] = header[i];
          }
          for(int i = 0; i < data_len + 1; i++){
            packet[i+6] = data[i];
          }
          pack_len = 7 + data_len;

          
          // data.resize(data_len + 1);     // Read CRC byte as well 
          // RCLCPP_INFO(this->get_logger(),"data_len: %d, seq_num: %d, data_len %d, cmd_id %d", (int)data.size(), seq_num, data_len, cmd_id);    //
          // int val = serial_driver_->port()->receive(data);
          // if(cmd_id==2) RCLCPP_INFO(this->get_logger(), "Val: %d",val);

          // serial_driver_->port()->receive(tail);
          // RCLCPP_INFO(this->get_logger(), "End: %d, %d",tail[0],tail[1]);
          // data.insert(data.begin(), header[0]);
          // ReceivePacket packet = fromVector(data);

          bool crc_ok = crc8::verify_crc8_check_sum(packet, pack_len);
          // unsigned char crc_sum = crc8::get_crc8_check_sum(packet,pack_len-1);
          // printf("Compare: %x, %x\n", packet[pack_len-1], crc_sum);
          // bool crc_ok =
          //   crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
          // if (crc_ok && *tail1=='E' && *tail2 == 'D') {
          // if (crc_ok) {
          if((*tail1=='E' && *tail2 == 'D') || crc_ok){
            // RCLCPP_INFO(this->get_logger(), "CRC OK");
            // RCLCPP_INFO(this->get_logger(),"seq_num: %d, data_len %d, cmd_id %d", seq_num, data_len, cmd_id);    //
            // if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
            //   setParam(rclcpp::Parameter("detect_color", packet.detect_color));
            //   previous_receive_color_ = packet.detect_color;
            // }

            // if (packet.reset_tracker) {
            //   resetTracker();
            // }
            // geometry_msgs::msg::TransformStamped t;
            // timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
            // t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
            // t.header.frame_id = "odom";
            // t.child_frame_id = "gimbal_link";
            // tf2::Quaternion q;
            // q.setRPY(packet.roll, packet.pitch, packet.yaw);
            // t.transform.rotation = tf2::toMsg(q);
            // tf_broadcaster_->sendTransform(t);

            // if (abs(packet.aim_x) > 0.01) {
            //   aiming_point_.header.stamp = this->now();
            //   aiming_point_.pose.position.x = packet.aim_x;
            //   aiming_point_.pose.position.y = packet.aim_y;
            //   aiming_point_.pose.position.z = packet.aim_z;
            //   marker_pub_->publish(aiming_point_);
            // }
            if(cmd_id == 0){
              gimbal_data_t gimbal;
              std::memcpy(&gimbal, &packet[6], 10);
              printf("Gimbal Pitch, Yaw, Mode, Debug: %f %f %d %d\n",gimbal.rel_yaw,gimbal.rel_pitch,gimbal.mode,gimbal.debug_int);
              geometry_msgs::msg::TransformStamped t;
              timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
              t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
              t.header.frame_id = "odom";
              t.child_frame_id = "gimbal_link";
              tf2::Quaternion q;
              q.setRPY(0.0, gimbal.rel_pitch, gimbal.rel_yaw);
              t.transform.rotation = tf2::toMsg(q);
              tf_broadcaster_->sendTransform(t);
            }else if(cmd_id == 1){
              color_data_t color;
              std::memcpy(&color, &packet[6], 1);
              printf("Color: %d\n", color.my_color);
            }else if(cmd_id == 2){
              chassis_data_t chassis;
              std::memcpy(&chassis, &packet[6],12);
              printf("Chassis: vx, vy, vw: %f,%f,%f\n",chassis.vx, chassis.vy, chassis.vw);
            }

          } else {
            RCLCPP_ERROR(get_logger(), "CRC error!");
          }
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
        }
      }  
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  const static std::map<std::string, uint8_t> id_unit8_map{
    {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  try {
    SendPacket packet;
    packet.tracking = msg->tracking;
    packet.id = id_unit8_map.at(msg->id);
    packet.armors_num = msg->armors_num;
    packet.x = msg->position.x;
    packet.y = msg->position.y;
    packet.z = msg->position.z;
    packet.yaw = msg->yaw;
    packet.vx = msg->velocity.x;
    packet.vy = msg->velocity.y;
    packet.vz = msg->velocity.z;
    packet.v_yaw = msg->v_yaw;
    packet.r1 = msg->radius_1;
    packet.r2 = msg->radius_2;
    packet.dz = msg->dz;
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    std::vector<uint8_t> data = toVector(packet);

    serial_driver_->port()->send(data);

    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
