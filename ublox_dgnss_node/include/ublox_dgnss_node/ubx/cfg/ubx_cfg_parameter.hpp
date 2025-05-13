// Copyright 2021 Australian Robotics Supplies & Technology
// Copyright 2023 G. Sokoll
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
 * @file ubx_cfg_parameter.hpp
 * @brief Definition of the UbxCfgParameter class for handling UBX-CFG parameters
 */

#ifndef UBLOX_DGNSS_NODE__UBX__CFG__UBX_CFG_PARAMETER_HPP_
#define UBLOX_DGNSS_NODE__UBX__CFG__UBX_CFG_PARAMETER_HPP_

#include <string>
#include <vector>
#include <map>
#include <optional>
#include <stdexcept>
#include <cstdint>
#include <memory>

#include "ublox_dgnss_node/ubx/ubx_cfg.hpp"

namespace ubx::cfg

/**
 * @brief Represents a behavior change in a parameter for a specific firmware version
 */
struct BehaviorChange {
  std::string version;       ///< Firmware version where behavior changed
  std::string description;   ///< Description of the behavior change
};

/**
 * @brief Represents firmware support information for a parameter on a specific device
 */
struct FirmwareSupport {
  std::string since;                      ///< Firmware version when parameter was introduced
  std::optional<std::string> until;       ///< Firmware version when parameter was deprecated (if applicable)
  std::vector<BehaviorChange> behavior_changes;  ///< List of behavior changes across firmware versions
};

/**
 * @brief Class representing a UBX-CFG parameter with all its attributes
 * 
 * This class encapsulates all the information about a UBX-CFG parameter,
 * including its name, key ID, type, scale, unit, applicable devices,
 * description, possible values, default value, and firmware support information.
 */
class UbxCfgParameter
{
public:
  /**
   * @brief Default constructor
   */
  UbxCfgParameter() = default;

  /**
   * @brief Constructor with all parameter attributes
   * 
   * @param name Parameter name
   * @param key_id Parameter key ID
   * @param type Parameter type
   * @param scale Parameter scale
   * @param unit Parameter unit
   * @param applicable_devices List of applicable device types
   * @param description Parameter description
   * @param group Parameter group
   * @param firmware_support Map of device type to firmware support information
   * @param possible_values Map of possible value names to values (for enum types)
   * @param default_value Default parameter value
   * @param min_value Minimum parameter value (optional)
   * @param max_value Maximum parameter value (optional)
   */
  UbxCfgParameter(
    const std::string & name,
    const ubx_key_id_t & key_id,
    const ubx_type_t & type,
    double scale,
    const ubx_unit_t & unit,
    const std::vector<std::string> & applicable_devices,
    const std::string & description,
    const std::string & group,
    const std::map<std::string, FirmwareSupport> & firmware_support,
    const std::map<std::string, std::string> & possible_values = {},
    const std::string & default_value = "",
    const std::optional<std::string> & min_value = std::nullopt,
    const std::optional<std::string> & max_value = std::nullopt);

  /**
   * @brief Get the parameter name
   * @return Parameter name
   */
  const std::string & get_name() const;

  /**
   * @brief Get the parameter key ID
   * @return Parameter key ID
   */
  const ubx_key_id_t & get_key_id() const;

  /**
   * @brief Get the parameter type
   * @return Parameter type
   */
  const ubx_type_t & get_type() const;

  /**
   * @brief Get the parameter scale
   * @return Parameter scale
   */
  double get_scale() const;

  /**
   * @brief Get the parameter unit
   * @return Parameter unit
   */
  const ubx_unit_t & get_unit() const;

  /**
   * @brief Get the applicable device types
   * @return List of applicable device types
   */
  const std::vector<std::string> & get_applicable_devices() const;

  /**
   * @brief Get the parameter description
   * @return Parameter description
   */
  const std::string & get_description() const;

  /**
   * @brief Get the parameter group
   * @return Parameter group
   */
  const std::string & get_group() const;

  /**
   * @brief Get the firmware support information
   * @return Map of device type to firmware support information
   */
  const std::map<std::string, FirmwareSupport> & get_firmware_support() const;

  /**
   * @brief Get the possible values (for enum types)
   * @return Map of possible value names to values
   */
  const std::map<std::string, std::string> & get_possible_values() const;

  /**
   * @brief Get the default value
   * @return Default value
   */
  const std::string & get_default_value() const;

  /**
   * @brief Get the minimum value
   * @return Minimum value (if defined)
   */
  const std::optional<std::string> & get_min_value() const;

  /**
   * @brief Get the maximum value
   * @return Maximum value (if defined)
   */
  const std::optional<std::string> & get_max_value() const;

  /**
   * @brief Check if the parameter is applicable to a specific device type
   * @param device_type Device type to check
   * @return True if the parameter is applicable to the device type
   */
  bool is_applicable_to_device(const std::string & device_type) const;

  /**
   * @brief Check if the parameter is supported in a specific firmware version
   * @param device_type Device type to check
   * @param firmware_version Firmware version to check
   * @return True if the parameter is supported in the firmware version
   */
  bool is_supported_in_firmware(
    const std::string & device_type,
    const std::string & firmware_version) const;

  /**
   * @brief Check if the parameter has behavior changes in a specific firmware version
   * @param device_type Device type to check
   * @param firmware_version Firmware version to check
   * @return Description of behavior change if present, empty string otherwise
   */
  std::string get_behavior_change(
    const std::string & device_type,
    const std::string & firmware_version) const;

  /**
   * @brief Convert a string value to a UBX value
   * @param value_str String value to convert
   * @return UBX value
   * @throw std::invalid_argument if the value is invalid
   */
  value_t string_to_ubx_value(const std::string & value_str) const;

  /**
   * @brief Convert a UBX value to a string value
   * @param value UBX value to convert
   * @return String value
   */
  std::string ubx_value_to_string(const value_t & value) const;

  /**
   * @brief Validate a string value against the parameter constraints
   * @param value_str String value to validate
   * @return True if the value is valid
   */
  bool validate_string_value(const std::string & value_str) const;

  /**
   * @brief Validate a UBX value against the parameter constraints
   * @param value UBX value to validate
   * @return True if the value is valid
   */
  bool validate_ubx_value(const value_t & value) const;

  /**
   * @brief Compare firmware versions
   * @param version1 First firmware version
   * @param version2 Second firmware version
   * @return -1 if version1 < version2, 0 if version1 == version2, 1 if version1 > version2
   */
  static int compare_firmware_versions(
    const std::string & version1,
    const std::string & version2);

private:
  std::string name_;                                  ///< Parameter name
  ubx_key_id_t key_id_;                               ///< Parameter key ID
  ubx_type_t type_;                                   ///< Parameter type
  double scale_;                                      ///< Parameter scale
  ubx_unit_t unit_;                                   ///< Parameter unit
  std::vector<std::string> applicable_devices_;       ///< List of applicable device types
  std::string description_;                           ///< Parameter description
  std::string group_;                                 ///< Parameter group
  std::map<std::string, FirmwareSupport> firmware_support_;  ///< Map of device type to firmware support information
  std::map<std::string, std::string> possible_values_;  ///< Map of possible value names to values (for enum types)
  std::string default_value_;                         ///< Default parameter value
  std::optional<std::string> min_value_;              ///< Minimum parameter value (optional)
  std::optional<std::string> max_value_;              ///< Maximum parameter value (optional)

  /**
   * @brief Parse a hexadecimal string to an integer
   * @param hex_str Hexadecimal string
   * @return Integer value
   * @throw std::invalid_argument if the string is not a valid hexadecimal number
   */
  static uint64_t parse_hex_string(const std::string & hex_str);
};

}  // namespace ubx::cfg

#endif  // UBLOX_DGNSS_NODE__UBX__CFG__UBX_CFG_PARAMETER_HPP_
