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
 * @file ubx_cfg_handler.hpp
 * @brief Definition of the UbxCfgHandler class for handling UBX-CFG operations
 */

#ifndef UBLOX_DGNSS_NODE__UBX__CFG__UBX_CFG_HANDLER_HPP_
#define UBLOX_DGNSS_NODE__UBX__CFG__UBX_CFG_HANDLER_HPP_

#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <optional>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "ublox_dgnss_node/usb.hpp"
#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_parameter_loader.hpp"
#include "ublox_dgnss_node/ubx/mon/ubx_mon_ver.hpp"

namespace ubx::cfg {

/**
 * @brief Class for handling UBX-CFG operations
 * 
 * This class is responsible for managing parameter operations, including
 * detecting firmware versions, filtering parameters based on device type and
 * firmware version, and handling parameter get/set operations.
 */
class UbxCfgHandler
{
public:
  /**
   * @brief Constructor with node, USB connection, device type, and parameter file path
   * @param node ROS node
   * @param usbc USB connection
   * @param device_type Device type
   * @param parameter_file_path Path to the parameter file
   */
  UbxCfgHandler(
    rclcpp::Node * node,
    std::shared_ptr<usb::Connection> usbc,
    const std::string & device_type,
    const std::string & parameter_file_path);

  /**
   * @brief Initialize the handler
   * @return True if initialization was successful
   */
  bool initialize();

  /**
   * @brief Handle parameter changes
   * @param parameters Vector of parameters that changed
   * @return Result of the parameter change
   */
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Get parameter value from device
   * @param name Parameter name
   * @return True if the parameter value was retrieved successfully
   */
  bool get_parameter_value(const std::string & name);

  /**
   * @brief Set parameter value on device
   * @param name Parameter name
   * @param value Parameter value
   * @return True if the parameter value was set successfully
   */
  bool set_parameter_value(const std::string & name, const rclcpp::ParameterValue & value);

  /**
   * @brief Get the device type
   * @return Device type
   */
  const std::string & get_device_type() const;

  /**
   * @brief Get the firmware version
   * @return Firmware version
   */
  const std::string & get_firmware_version() const;

  /**
   * @brief Get the parameter loader
   * @return Parameter loader
   */
  const UbxCfgParameterLoader & get_parameter_loader() const;

  /**
   * @brief Get the parameters applicable to the current device and firmware
   * @return Vector of applicable parameters
   */
  std::vector<UbxCfgParameter> get_applicable_parameters() const;

  /**
   * @brief Check if a parameter is applicable to the current device and firmware
   * @param name Parameter name
   * @return True if the parameter is applicable
   */
  bool is_parameter_applicable(const std::string & name) const;

  /**
   * @brief Check if a parameter has behavior changes in the current firmware
   * @param name Parameter name
   * @return Description of behavior change if present, empty string otherwise
   */
  std::string get_parameter_behavior_change(const std::string & name) const;

private:
  /**
   * @brief Detect the device firmware version
   * @return Detected firmware version
   */
  std::string detect_firmware_version();

  /**
   * @brief Register parameters with ROS
   * @return True if parameters were registered successfully
   */
  bool register_parameters();

  /**
   * @brief Convert a ROS parameter value to a UBX value
   * @param param UBX parameter
   * @param value ROS parameter value
   * @return UBX value
   */
  ::ubx::value_t ros_to_ubx_value(
    const UbxCfgParameter & param,
    const rclcpp::ParameterValue & value);

  /**
   * @brief Convert a UBX value to a ROS parameter value
   * @param param UBX parameter
   * @param value UBX value
   * @return ROS parameter value
   */
  rclcpp::ParameterValue ubx_to_ros_value(
    const UbxCfgParameter & param,
    const ::ubx::value_t & value);

  rclcpp::Node * node_;                                  ///< ROS node
  std::shared_ptr<usb::Connection> usbc_;                ///< USB connection
  std::string device_type_;                              ///< Device type
  std::string firmware_version_;                         ///< Firmware version
  UbxCfgParameterLoader parameter_loader_;               ///< Parameter loader
  std::unordered_map<std::string, UbxCfgParameter> registered_parameters_;  ///< Map of registered parameter names to parameters
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;  ///< Parameter callback handle
};

}  // namespace ubx::cfg

#endif  // UBLOX_DGNSS_NODE__UBX__CFG__UBX_CFG_HANDLER_HPP_
