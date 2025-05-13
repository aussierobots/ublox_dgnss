// Copyright 2025 GreenForge Labs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file ubx_cfg_handler.cpp
 * @brief Implementation of the UbxCfgHandler class for handling UBX-CFG operations
 */

#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_handler.hpp"

#include <regex>
#include <algorithm>
#include <sstream>

#include "ublox_dgnss_node/ubx/ubx_cfg.hpp"
#include "ublox_dgnss_node/ubx/ubx_types.hpp"
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/mon/ubx_mon_ver.hpp"
#include "ublox_dgnss_node/ubx/ubx_ack.hpp"

namespace ubx::cfg {

UbxCfgHandler::UbxCfgHandler(
  rclcpp::Node * node,
  std::shared_ptr<usb::Connection> usbc,
  const std::string & device_type,
  const std::string & parameter_file_path)
: node_(node),
  usbc_(usbc),
  device_type_(device_type),
  parameter_loader_(parameter_file_path)
{
}

bool UbxCfgHandler::initialize()
{
  try {
    // Load parameters from file
    if (!parameter_loader_.load()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to load parameters from file: %s", parameter_loader_.get_file_path().c_str());
      return false;
    }

    // Detect firmware version
    firmware_version_ = detect_firmware_version();
    RCLCPP_INFO(
      node_->get_logger(),
      "Detected firmware version: %s", firmware_version_.c_str());

    // Register parameters with ROS
    if (!register_parameters()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to register parameters");
      return false;
    }

    // Register parameter callback
    parameter_callback_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&UbxCfgHandler::on_parameter_change, this, std::placeholders::_1));

    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Error initializing UbxCfgHandler: %s", e.what());
    return false;
  }
}

::rcl_interfaces::msg::SetParametersResult UbxCfgHandler::on_parameter_change(
  const std::vector<rclcpp::Parameter> & parameters)
{
  ::rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto & param : parameters) {
    // Check if this is a UBX parameter
    auto it = registered_parameters_.find(param.get_name());
    if (it != registered_parameters_.end()) {
      const auto & ubx_param = it->second;

      try {
        // Get the parameter value as a ParameterValue object
        rclcpp::ParameterValue param_value = param.get_parameter_value();
        
        // Convert ROS parameter value to UBX value
        ::ubx::value_t ubx_value = ros_to_ubx_value(ubx_param, param_value);

        // Validate the value
        if (!ubx_param.validate_ubx_value(ubx_value)) {
          result.successful = false;
          result.reason = "Invalid value for parameter: " + param.get_name();
          break;
        }

        // Set the parameter directly using our set_parameter_value method
        if (!set_parameter_value(param.get_name(), param_value)) {
          result.successful = false;
          result.reason = "Failed to set parameter on device: " + param.get_name();
          break;
        }

        RCLCPP_DEBUG(
          node_->get_logger(),
          "Set parameter %s to %s", param.get_name().c_str(), param.value_to_string().c_str());
      } catch (const std::exception & e) {
        result.successful = false;
        result.reason = "Error setting parameter " + param.get_name() + ": " + e.what();
        break;
      }
    }
  }

  return result;
}

bool UbxCfgHandler::get_parameter_value(const std::string & name)
{
  // Check if this is a registered UBX parameter
  auto it = registered_parameters_.find(name);
  if (it == registered_parameters_.end()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Parameter not found: %s", name.c_str());
    return false;
  }

  const auto & ubx_param = it->second;

  try {
    // Create a CFG-VALGET message to get the parameter value
    auto payload = std::make_shared<ubx::cfg::CfgValGetPayload>();
    payload->layer = 1;  // RAM layer
    payload->position = 0;  // Start from first position
    payload->keys.push_back(ubx_param.get_key_id());
    
    // Create a frame for the message
    auto frame_poll = std::make_shared<ubx::FramePoll>();
    frame_poll->msg_class = ubx::cfg::CfgValGetPayload::MSG_CLASS;
    frame_poll->msg_id = ubx::cfg::CfgValGetPayload::MSG_ID;
    
    // Get the payload data
    auto [payload_data, payload_size] = payload->make_poll_payload();
    
    // Set up the frame
    frame_poll->payload = reinterpret_cast<ch_t *>(payload_data);
    frame_poll->length = payload_size;
    
    // Calculate checksum and build frame buffer
    std::tie(frame_poll->ck_a, frame_poll->ck_b) = frame_poll->ubx_check_sum();
    frame_poll->build_frame_buf();

    // Send the message
    usbc_->write_buffer(frame_poll->buf.data(), frame_poll->buf.size());

    // Read the response
    std::vector<u1_t> buffer(1024);
    int bytes_read = usbc_->read_chars(buffer.data(), buffer.size());
    
    if (bytes_read <= 0) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to read response from device");
      return false;
    }
    
    // Find the UBX message in the buffer
    size_t i = 0;
    while (i < static_cast<size_t>(bytes_read) - 1) {
      if (buffer[i] == 0xB5 && buffer[i+1] == 0x62) {  // UBX sync chars
        break;
      }
      i++;
    }
    
    if (i >= static_cast<size_t>(bytes_read) - 1) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to find UBX message in response");
      return false;
    }
    
    // Check if this is a CFG-VALGET message
    if (buffer[i+2] != ubx::cfg::CfgValGetPayload::MSG_CLASS || 
        buffer[i+3] != ubx::cfg::CfgValGetPayload::MSG_ID) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Unexpected message type: 0x%02X 0x%02X",
        buffer[i+2], buffer[i+3]);
      return false;
    }
    
    // Get the payload length
    u2_t length = *reinterpret_cast<u2_t *>(&buffer[i+4]);
    
    // Parse the response using the CfgValGetPayload constructor
    ubx::cfg::CfgValGetPayload response_payload(&buffer[i+6], length);
    
    // Check if we got a valid response
    if (response_payload.cfg_data.empty()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "No parameter data received for %s",
        name.c_str());
      return false;
    }
    
    // Extract the parameter value
    auto value = response_payload.cfg_data[0].ubx_value;
    
    // Convert the UBX value to a ROS parameter value
    rclcpp::ParameterValue ros_value = ubx_to_ros_value(ubx_param, value);
    
    // Set the parameter in the node
    node_->set_parameter(rclcpp::Parameter(name, ros_value));

    RCLCPP_DEBUG(
      node_->get_logger(),
      "Got parameter %s = %s", name.c_str(), node_->get_parameter(name).value_to_string().c_str());
    

    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Error getting parameter %s: %s", name.c_str(), e.what());
    return false;
  }
}

bool UbxCfgHandler::set_parameter_value(const std::string & name, const rclcpp::ParameterValue & value)
{
  // Check if this is a registered UBX parameter
  auto it = registered_parameters_.find(name);
  if (it == registered_parameters_.end()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Parameter not found: %s", name.c_str());
    return false;
  }

  const auto & ubx_param = it->second;

  try {
    // Convert ROS parameter value to UBX value
    ::ubx::value_t ubx_value = ros_to_ubx_value(ubx_param, value);

    // Validate the value
    if (!ubx_param.validate_ubx_value(ubx_value)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Invalid value for parameter: %s", name.c_str());
      return false;
    }

    // Create a CFG-VALSET message to set the parameter value
    auto payload = std::make_shared<ubx::cfg::CfgValSetPayload>();
    payload->layers.bits.ram = 1;    // Set RAM layer
    payload->layers.bits.bbr = 1;    // Set BBR layer
    payload->layers.bits.flash = 1;  // Set Flash layer
    
    // Add the key-value pair to the payload
    ubx::cfg::key_value_t kv;
    kv.ubx_key_id = ubx_param.get_key_id();
    kv.ubx_value = ubx_value;
    payload->cfg_data.push_back(kv);
    
    // Create a frame for the message
    auto frame = std::make_shared<ubx::FrameValSet>();
    frame->msg_class = ubx::cfg::CfgValSetPayload::MSG_CLASS;
    frame->msg_id = ubx::cfg::CfgValSetPayload::MSG_ID;
    
    // Get the payload data
    auto [payload_data, payload_size] = payload->make_poll_payload();
    
    // Set up the frame
    frame->payload = reinterpret_cast<ch_t *>(payload_data);
    frame->length = payload_size;
    
    // Calculate checksum and build frame buffer
    std::tie(frame->ck_a, frame->ck_b) = frame->ubx_check_sum();
    frame->build_frame_buf();

    // Send the message
    usbc_->write_buffer(frame->buf.data(), frame->buf.size());

    // Read the ACK/NAK response
    std::vector<u1_t> buffer(1024);
    int bytes_read = usbc_->read_chars(buffer.data(), buffer.size());
    if (bytes_read <= 0) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "No response to CFG-VALSET message for parameter: %s", name.c_str());
      return false;
    }

    // Parse the response
    // First, find the UBX message in the buffer
    size_t i = 0;
    while (i < static_cast<size_t>(bytes_read - 1)) {
      if (buffer[i] == 0xB5 && buffer[i+1] == 0x62) {  // UBX sync chars
        break;
      }
      i++;
    }
    
    if (i >= static_cast<size_t>(bytes_read - 1)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to find UBX message in response");
      return false;
    }
    
    // Parse the ACK-ACK or ACK-NAK message
    ubx::ack::AckNakPayload ack_nak(&buffer[i+6], 2);  // Skip header (6 bytes) to get to payload

    // Check for ACK response (ACK-ACK has class 0x05, ID 0x01)
    if (buffer[i+2] == 0x05 && buffer[i+3] == 0x01) {
      // Update the ROS parameter if the device update was successful
      node_->set_parameter(rclcpp::Parameter(name, value));
      
      RCLCPP_DEBUG(
        node_->get_logger(),
        "Successfully set parameter %s",
        name.c_str());
      
      return true;
    } else if (buffer[i+2] == 0x05 && buffer[i+3] == 0x00)  // ACK-NAK has class 0x05, ID 0x00
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to set parameter %s: NAK received", name.c_str());
      return false;
    } else {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Unexpected response to CFG-VALSET message for parameter: %s", name.c_str());
      return false;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Error setting parameter %s: %s", name.c_str(), e.what());
    return false;
  }
}

const std::string & UbxCfgHandler::get_device_type() const
{
  return device_type_;
}

const std::string & UbxCfgHandler::get_firmware_version() const
{
  return firmware_version_;
}

const UbxCfgParameterLoader & UbxCfgHandler::get_parameter_loader() const
{
  return parameter_loader_;
}

std::vector<UbxCfgParameter> UbxCfgHandler::get_applicable_parameters() const
{
  return parameter_loader_.get_parameters_for_device_and_firmware(device_type_, firmware_version_);
}

bool UbxCfgHandler::is_parameter_applicable(const std::string & name) const
{
  auto param_opt = parameter_loader_.get_parameter_by_name(name);
  if (!param_opt.has_value()) {
    return false;
  }

  const auto & param = param_opt.value();
  return param.is_applicable_to_device(device_type_) &&
         param.is_supported_in_firmware(device_type_, firmware_version_);
}

std::string UbxCfgHandler::get_parameter_behavior_change(const std::string & name) const
{
  auto param_opt = parameter_loader_.get_parameter_by_name(name);
  if (!param_opt.has_value()) {
    return "";
  }

  const auto & param = param_opt.value();
  return param.get_behavior_change(device_type_, firmware_version_);
}

std::string UbxCfgHandler::detect_firmware_version()
{
  try {
    // Create a MON-VER poll message
    auto frame_poll = std::make_shared<ubx::FramePoll>();
    frame_poll->msg_class = ubx::mon::ver::MonVerPayload::MSG_CLASS;
    frame_poll->msg_id = ubx::mon::ver::MonVerPayload::MSG_ID;
    frame_poll->payload = nullptr;  // Poll messages have empty payload
    frame_poll->length = 0;
    
    // Calculate checksum and build frame buffer
    std::tie(frame_poll->ck_a, frame_poll->ck_b) = frame_poll->ubx_check_sum();
    frame_poll->build_frame_buf();

    // Send the message
    usbc_->write_buffer(frame_poll->buf.data(), frame_poll->buf.size());

    // Read the response
    std::vector<u1_t> buffer(1024);
    int bytes_read = usbc_->read_chars(buffer.data(), buffer.size());
    
    if (bytes_read <= 0) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to read response from device");
      return "";
    }
    
    // Find the UBX message in the buffer
    size_t i = 0;
    while (i < static_cast<size_t>(bytes_read) - 1) {
      if (buffer[i] == 0xB5 && buffer[i+1] == 0x62) {  // UBX sync chars
        break;
      }
      i++;
    }
    
    if (i >= static_cast<size_t>(bytes_read) - 1) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to find UBX message in response");
      return "";
    }
    
    // Check if this is a MON-VER message
    if (buffer[i+2] != ubx::mon::ver::MonVerPayload::MSG_CLASS || 
        buffer[i+3] != ubx::mon::ver::MonVerPayload::MSG_ID) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Unexpected message type: 0x%02X 0x%02X",
        buffer[i+2], buffer[i+3]);
      return "";
    }
    
    // Get the payload length
    u2_t length = *reinterpret_cast<u2_t *>(&buffer[i+4]);
    
    // Parse the response using the MonVerPayload constructor
    ubx::mon::ver::MonVerPayload mon_ver(&buffer[i+6], length);
    
    // Extract the firmware version (sw_version is a char array, not a string)
    std::string fw_version(reinterpret_cast<char *>(mon_ver.sw_version));
    firmware_version_ = fw_version;
    
    RCLCPP_INFO(
      node_->get_logger(),
      "Detected firmware version: %s", fw_version.c_str());
    
    return fw_version;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Error detecting firmware version: %s", e.what());
    return "";
  }
}

bool UbxCfgHandler::register_parameters()
{
  // Get parameters applicable to the current device and firmware
  auto applicable_params = get_applicable_parameters();
  
  if (applicable_params.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "No applicable parameters found for device %s with firmware %s",
      device_type_.c_str(), firmware_version_.c_str());
    return false;
  }
  
  RCLCPP_INFO(
    node_->get_logger(),
    "Registering %zu parameters for device %s with firmware %s",
    applicable_params.size(), device_type_.c_str(), firmware_version_.c_str());
  
  // Register each parameter
  for (const auto & param : applicable_params) {
    try {
      // Create parameter name
      std::string param_name = param.get_name();
      
      // Check for behavior changes
      std::string behavior_change = param.get_behavior_change(device_type_, firmware_version_);
      if (!behavior_change.empty()) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Parameter %s has behavior changes in firmware %s: %s",
          param_name.c_str(), firmware_version_.c_str(), behavior_change.c_str());
      }
      
      // Register the parameter with ROS
      auto param_descriptor = rcl_interfaces::msg::ParameterDescriptor();
      param_descriptor.name = param.get_name();
      param_descriptor.description = param.get_description();
      param_descriptor.read_only = false;

      // Set the parameter type based on the UBX type
      switch (param.get_type()) {
        case ::ubx::U1:
        case ::ubx::U2:
        case ::ubx::U4:
        case ::ubx::I1:
        case ::ubx::I2:
        case ::ubx::I4:
          node_->declare_parameter(param.get_name(), 0, param_descriptor);
          break;
        case ::ubx::R4:
        case ::ubx::R8:
          node_->declare_parameter(param.get_name(), 0.0, param_descriptor);
          break;
        case ::ubx::L:
          node_->declare_parameter(param.get_name(), false, param_descriptor);
          break;
        case ::ubx::X1:
        case ::ubx::X2:
        case ::ubx::X4:
        case ::ubx::E1:
        case ::ubx::E2:
        case ::ubx::E4:
          // For enum and bitfield types, use string representation
          node_->declare_parameter(param.get_name(), "", param_descriptor);
          break;
        default:
          RCLCPP_WARN(
            node_->get_logger(),
            "Unsupported parameter type for parameter: %s",
            param.get_name().c_str());
          return false;
      }
      
      // Store the parameter for later use
      registered_parameters_[param_name] = param;
      
      RCLCPP_DEBUG(
        node_->get_logger(),
        "Registered parameter: %s", param_name.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Error registering parameter %s: %s", param.get_name().c_str(), e.what());
    }
  }
  
  return !registered_parameters_.empty();
}

::ubx::value_t UbxCfgHandler::ros_to_ubx_value(
  const UbxCfgParameter & param,
  const rclcpp::ParameterValue & value)
{
  ::ubx::value_t ubx_value = {}; // Initialize all fields to zero

  // Handle different parameter types
  switch (param.get_type()) {
    case ::ubx::U1:
      if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        ubx_value.u1 = static_cast<::ubx::u1_t>(value.get<int64_t>());
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        ubx_value.u1 = static_cast<::ubx::u1_t>(value.get<double>());
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        // Try to parse the string as a number
        try {
          ubx_value.u1 = static_cast<::ubx::u1_t>(std::stoul(value.get<std::string>()));
        } catch (const std::exception & e) {
          throw std::runtime_error("Failed to parse integer value: " + value.get<std::string>());
        }
      } else {
        throw std::runtime_error("Invalid parameter type for U1");
      }
      break;

    case ::ubx::U2:
      if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        ubx_value.u2 = static_cast<::ubx::u2_t>(value.get<int64_t>());
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        ubx_value.u2 = static_cast<::ubx::u2_t>(value.get<double>());
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        try {
          ubx_value.u2 = static_cast<::ubx::u2_t>(std::stoul(value.get<std::string>()));
        } catch (const std::exception & e) {
          throw std::runtime_error("Failed to parse integer value: " + value.get<std::string>());
        }
      } else {
        throw std::runtime_error("Invalid parameter type for U2");
      }
      break;

    case ::ubx::U4:
      if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        ubx_value.u4 = static_cast<::ubx::u4_t>(value.get<int64_t>());
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        ubx_value.u4 = static_cast<::ubx::u4_t>(value.get<double>());
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        try {
          ubx_value.u4 = static_cast<::ubx::u4_t>(std::stoul(value.get<std::string>()));
        } catch (const std::exception & e) {
          throw std::runtime_error("Failed to parse integer value: " + value.get<std::string>());
        }
      } else {
        throw std::runtime_error("Invalid parameter type for U4");
      }
      break;

    case ::ubx::I1:
      if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        ubx_value.i1 = static_cast<::ubx::i1_t>(value.get<int64_t>());
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        ubx_value.i1 = static_cast<::ubx::i1_t>(value.get<double>());
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        try {
          ubx_value.i1 = static_cast<::ubx::i1_t>(std::stol(value.get<std::string>()));
        } catch (const std::exception & e) {
          throw std::runtime_error("Failed to parse integer value: " + value.get<std::string>());
        }
      } else {
        throw std::runtime_error("Invalid parameter type for I1");
      }
      break;

    case ::ubx::I2:
      if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        ubx_value.i2 = static_cast<::ubx::i2_t>(value.get<int64_t>());
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        ubx_value.i2 = static_cast<::ubx::i2_t>(value.get<double>());
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        try {
          ubx_value.i2 = static_cast<::ubx::i2_t>(std::stol(value.get<std::string>()));
        } catch (const std::exception & e) {
          throw std::runtime_error("Failed to parse integer value: " + value.get<std::string>());
        }
      } else {
        throw std::runtime_error("Invalid parameter type for I2");
      }
      break;

    case ::ubx::I4:
      if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        ubx_value.i4 = static_cast<::ubx::i4_t>(value.get<int64_t>());
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        ubx_value.i4 = static_cast<::ubx::i4_t>(value.get<double>());
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        try {
          ubx_value.i4 = static_cast<::ubx::i4_t>(std::stol(value.get<std::string>()));
        } catch (const std::exception & e) {
          throw std::runtime_error("Failed to parse integer value: " + value.get<std::string>());
        }
      } else {
        throw std::runtime_error("Invalid parameter type for I4");
      }
      break;

    case ::ubx::R4:
      if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        ubx_value.r4 = static_cast<::ubx::r4_t>(value.get<double>());
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        ubx_value.r4 = static_cast<::ubx::r4_t>(value.get<int64_t>());
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        try {
          ubx_value.r4 = static_cast<::ubx::r4_t>(std::stod(value.get<std::string>()));
        } catch (const std::exception & e) {
          throw std::runtime_error("Failed to parse floating-point value: " + value.get<std::string>());
        }
      } else {
        throw std::runtime_error("Invalid parameter type for R4");
      }
      break;

    case ::ubx::R8:
      if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        ubx_value.r8 = static_cast<::ubx::r8_t>(value.get<double>());
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        ubx_value.r8 = static_cast<::ubx::r8_t>(value.get<int64_t>());
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        try {
          ubx_value.r8 = static_cast<::ubx::r8_t>(std::stod(value.get<std::string>()));
        } catch (const std::exception & e) {
          throw std::runtime_error("Failed to parse floating-point value: " + value.get<std::string>());
        }
      } else {
        throw std::runtime_error("Invalid parameter type for R8");
      }
      break;

    case ::ubx::L:
      if (value.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        ubx_value.l = value.get<bool>() ? 1 : 0;
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        std::string str = value.get<std::string>();
        std::transform(str.begin(), str.end(), str.begin(), ::tolower);
        if (str == "true" || str == "1" || str == "on" || str == "yes") {
          ubx_value.l = 1;
        } else if (str == "false" || str == "0" || str == "off" || str == "no") {
          ubx_value.l = 0;
        } else {
          throw std::runtime_error("Invalid boolean value: " + str);
        }
      } else {
        throw std::runtime_error("Invalid parameter type for L");
      }
      break;

    case ::ubx::X1:
    case ::ubx::E1:
      if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        std::string str = value.get<std::string>();
        try {
          // Check if the string is a hex value (starts with 0x)
          if (str.substr(0, 2) == "0x") {
            ubx_value.x1 = static_cast<::ubx::x1_t>(std::stoul(str.substr(2), nullptr, 16));
          } else {
            ubx_value.x1 = static_cast<::ubx::x1_t>(std::stoul(str));
          }
        } catch (const std::exception & e) {
          throw std::runtime_error("Failed to parse hex value: " + str);
        }
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        ubx_value.x1 = static_cast<::ubx::x1_t>(value.get<int64_t>());
      } else {
        throw std::runtime_error("Invalid parameter type for X1/E1");
      }
      break;

    case ::ubx::X2:
    case ::ubx::E2:
      if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        std::string str = value.get<std::string>();
        try {
          // Check if the string is a hex value (starts with 0x)
          if (str.substr(0, 2) == "0x") {
            ubx_value.x2 = static_cast<::ubx::x2_t>(std::stoul(str.substr(2), nullptr, 16));
          } else {
            ubx_value.x2 = static_cast<::ubx::x2_t>(std::stoul(str));
          }
        } catch (const std::exception & e) {
          throw std::runtime_error("Failed to parse hex value: " + str);
        }
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        ubx_value.x2 = static_cast<::ubx::x2_t>(value.get<int64_t>());
      } else {
        throw std::runtime_error("Invalid parameter type for X2/E2");
      }
      break;

    case ::ubx::X4:
    case ::ubx::E4:
      if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        std::string str = value.get<std::string>();
        try {
          // Check if the string is a hex value (starts with 0x)
          if (str.substr(0, 2) == "0x") {
            ubx_value.x4 = static_cast<::ubx::x4_t>(std::stoul(str.substr(2), nullptr, 16));
          } else {
            ubx_value.x4 = static_cast<::ubx::x4_t>(std::stoul(str));
          }
        } catch (const std::exception & e) {
          throw std::runtime_error("Failed to parse hex value: " + str);
        }
      } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        ubx_value.x4 = static_cast<::ubx::x4_t>(value.get<int64_t>());
      } else {
        throw std::runtime_error("Invalid parameter type for X4/E4");
      }
      break;

    default:
      throw std::runtime_error("Unsupported parameter type: " + std::to_string(static_cast<int>(param.get_type())));
  }

  return ubx_value;
}

rclcpp::ParameterValue UbxCfgHandler::ubx_to_ros_value(
  const UbxCfgParameter & param,
  const ::ubx::value_t & value)
{
  // Handle different parameter types
  switch (param.get_type()) {
    case ::ubx::U1:
      return rclcpp::ParameterValue(static_cast<int64_t>(value.u1));

    case ::ubx::U2:
      return rclcpp::ParameterValue(static_cast<int64_t>(value.u2));

    case ::ubx::U4:
      return rclcpp::ParameterValue(static_cast<int64_t>(value.u4));

    case ::ubx::I1:
      return rclcpp::ParameterValue(static_cast<int64_t>(value.i1));

    case ::ubx::I2:
      return rclcpp::ParameterValue(static_cast<int64_t>(value.i2));

    case ::ubx::I4:
      return rclcpp::ParameterValue(static_cast<int64_t>(value.i4));

    case ::ubx::R4:
      return rclcpp::ParameterValue(static_cast<double>(value.r4));

    case ::ubx::R8:
      return rclcpp::ParameterValue(static_cast<double>(value.r8));

    case ::ubx::L:
      return rclcpp::ParameterValue(static_cast<bool>(value.l));

    case ::ubx::X1:
    case ::ubx::X2:
    case ::ubx::X4:
    case ::ubx::E1:
    case ::ubx::E2:
    case ::ubx::E4:
      // For enum and bitfield types, convert to a string representation
      {
        std::ostringstream ss;
        if (param.get_type() == ::ubx::X1 || param.get_type() == ::ubx::E1) {
          ss << "0x" << std::hex << static_cast<int>(value.x1);
        } else if (param.get_type() == ::ubx::X2 || param.get_type() == ::ubx::E2) {
          ss << "0x" << std::hex << static_cast<int>(value.x2);
        } else if (param.get_type() == ::ubx::X4 || param.get_type() == ::ubx::E4) {
          ss << "0x" << std::hex << static_cast<int>(value.x4);
        }
        return rclcpp::ParameterValue(ss.str());
      }

    default:
      RCLCPP_ERROR(
        node_->get_logger(),
        "Unsupported parameter type %d for parameter: %s",
        static_cast<int>(param.get_type()),
        param.get_name().c_str());
      return rclcpp::ParameterValue("");
  }
}

}  // namespace ubx::cfg
