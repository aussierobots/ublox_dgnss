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

#include "ublox_dgnss_node/ubx/cfg/val_get.hpp"
#include "ublox_dgnss_node/ubx/cfg/val_set.hpp"

namespace ubx::cfg

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

rcl_interfaces::msg::SetParametersResult UbxCfgHandler::on_parameter_change(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    // Check if this is a UBX parameter
    auto it = registered_parameters_.find(param.get_name());
    if (it != registered_parameters_.end()) {
      const auto & ubx_param = it->second;

      try {
        // Convert ROS parameter value to UBX value
        value_t ubx_value = ros_to_ubx_value(ubx_param, param.get_value());

        // Validate the value
        if (!ubx_param.validate_ubx_value(ubx_value)) {
          result.successful = false;
          result.reason = "Invalid value for parameter: " + param.get_name();
          break;
        }

        // Set the parameter value on the device
        auto payload = std::make_shared<CfgValSetPayload>();
        payload->add_key_value(ubx_param.get_key_id(), ubx_value);

        // Send the message
        if (!usbc_->send_ubx_msg(payload)) {
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
    // Create a payload to get the parameter value
    auto payload = std::make_shared<CfgValGetPayload>();
    payload->add_key(ubx_param.get_key_id());

    // Send the message and wait for response
    if (!usbc_->poll_ubx_msg(payload)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to get parameter value from device: %s", name.c_str());
      return false;
    }

    // Get the value from the response
    if (payload->get_num_values() == 0) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "No value returned for parameter: %s", name.c_str());
      return false;
    }

    // Get the value
    value_t value = payload->get_values()[0].second;

    // Convert to ROS parameter value and set it
    rclcpp::ParameterValue ros_value = ubx_to_ros_value(ubx_param, value);
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

  // Set the parameter
  try {
    node_->set_parameter(rclcpp::Parameter(name, value));
    return true;
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
  // Use UBX-MON-VER message to query firmware version
  auto ver_payload = std::make_shared<mon::ver::MonVerPayload>();
  
  if (!usbc_->poll_ubx_msg(ver_payload)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Failed to get firmware version, using default version");
    
    // Use default version based on device type
    if (device_type_ == "ZED-F9P") {
      return "HPG 1.32";
    } else if (device_type_ == "ZED-F9R") {
      return "HPS 1.30";
    } else {
      return "UNKNOWN";
    }
  }
  
  // Parse firmware version from sw_version field
  // Format is typically "EXT CORE 1.00 (12345678)"
  std::string sw_version = ver_payload->sw_version;
  
  // Extract firmware type and version
  // e.g., "HPG 1.32" or "HPS 1.30"
  std::regex pattern("([A-Z]+)\\s+(\\d+\\.\\d+)");
  std::smatch matches;
  if (std::regex_search(sw_version, matches, pattern)) {
    return matches[1].str() + " " + matches[2].str();
  }
  
  // If we couldn't parse the version, use a default based on device type
  if (device_type_ == "ZED-F9P") {
    return "HPG 1.32";
  } else if (device_type_ == "ZED-F9R") {
    return "HPS 1.30";
  } else {
    return "UNKNOWN";
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
      
      // Create parameter descriptor
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.name = param_name;
      descriptor.description = param.get_description();
      
      // Set parameter type and default value based on UBX type
      rclcpp::ParameterValue default_value;
      
      switch (param.get_type()) {
        case ubx_type_t::L:
          default_value = rclcpp::ParameterValue(false);
          break;
        case ubx_type_t::U1:
        case ubx_type_t::U2:
        case ubx_type_t::U4:
        case ubx_type_t::I1:
        case ubx_type_t::I2:
        case ubx_type_t::I4:
          default_value = rclcpp::ParameterValue(0);
          break;
        case ubx_type_t::R4:
        case ubx_type_t::R8:
          default_value = rclcpp::ParameterValue(0.0);
          break;
        case ubx_type_t::E1:
        case ubx_type_t::X1:
        case ubx_type_t::X2:
        case ubx_type_t::X4:
          default_value = rclcpp::ParameterValue("");
          break;
        default:
          RCLCPP_WARN(
            node_->get_logger(),
            "Unsupported parameter type for %s, skipping", param_name.c_str());
          continue;
      }
      
      // Declare the parameter
      node_->declare_parameter(param_name, default_value, descriptor);
      
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

value_t UbxCfgHandler::ros_to_ubx_value(
  const UbxCfgParameter & param,
  const rclcpp::ParameterValue & value)
{
  switch (param.get_type()) {
    case ubx_type_t::L:
      {
        value_t v;
        v.l = value.get<bool>();
        return v;
      }
    case ubx_type_t::U1:
      {
        value_t v;
        v.u1 = static_cast<uint8_t>(value.get<int>());
        return v;
      }
    case ubx_type_t::U2:
      {
        value_t v;
        v.u2 = static_cast<uint16_t>(value.get<int>());
        return v;
      }
    case ubx_type_t::U4:
      {
        value_t v;
        v.u4 = static_cast<uint32_t>(value.get<int>());
        return v;
      }
    case ubx_type_t::I1:
      {
        value_t v;
        v.i1 = static_cast<int8_t>(value.get<int>());
        return v;
      }
    case ubx_type_t::I2:
      {
        value_t v;
        v.i2 = static_cast<int16_t>(value.get<int>());
        return v;
      }
    case ubx_type_t::I4:
      {
        value_t v;
        v.i4 = static_cast<int32_t>(value.get<int>());
        return v;
      }
    case ubx_type_t::R4:
      {
        value_t v;
        v.r4 = static_cast<float>(value.get<double>());
        return v;
      }
    case ubx_type_t::R8:
      {
        value_t v;
        v.r8 = value.get<double>();
        return v;
      }
    case ubx_type_t::E1:
    case ubx_type_t::X1:
    case ubx_type_t::X2:
    case ubx_type_t::X4:
      {
        // For enum and hex types, convert from string
        return param.string_to_ubx_value(value.get<std::string>());
      }
    default:
      throw std::invalid_argument("Unsupported parameter type");
  }
}

rclcpp::ParameterValue UbxCfgHandler::ubx_to_ros_value(
  const UbxCfgParameter & param,
  const value_t & value)
{
  switch (param.get_type()) {
    case ubx_type_t::L:
      return rclcpp::ParameterValue(value.l);
    case ubx_type_t::U1:
      return rclcpp::ParameterValue(static_cast<int>(value.u1));
    case ubx_type_t::U2:
      return rclcpp::ParameterValue(static_cast<int>(value.u2));
    case ubx_type_t::U4:
      return rclcpp::ParameterValue(static_cast<int>(value.u4));
    case ubx_type_t::I1:
      return rclcpp::ParameterValue(static_cast<int>(value.i1));
    case ubx_type_t::I2:
      return rclcpp::ParameterValue(static_cast<int>(value.i2));
    case ubx_type_t::I4:
      return rclcpp::ParameterValue(static_cast<int>(value.i4));
    case ubx_type_t::R4:
      return rclcpp::ParameterValue(static_cast<double>(value.r4));
    case ubx_type_t::R8:
      return rclcpp::ParameterValue(value.r8);
    case ubx_type_t::E1:
    case ubx_type_t::X1:
    case ubx_type_t::X2:
    case ubx_type_t::X4:
      // For enum and hex types, convert to string
      return rclcpp::ParameterValue(param.ubx_value_to_string(value));
    default:
      throw std::invalid_argument("Unsupported parameter type");
  }
}

}  // namespace ubx::cfg
