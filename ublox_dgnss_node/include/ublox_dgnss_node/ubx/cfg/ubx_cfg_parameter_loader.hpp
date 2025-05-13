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
 * @file ubx_cfg_parameter_loader.hpp
 * @brief Definition of the UbxCfgParameterLoader class for loading UBX-CFG parameters from JSON files
 */

#ifndef UBLOX_DGNSS_NODE__UBX__CFG__UBX_CFG_PARAMETER_LOADER_HPP_
#define UBLOX_DGNSS_NODE__UBX__CFG__UBX_CFG_PARAMETER_LOADER_HPP_

#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <optional>
#include <memory>
#include <stdexcept>

#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_parameter.hpp"

// Forward declaration for nlohmann::json
namespace nlohmann
{
class json;
}

namespace ubx::cfg

/**
 * @brief Exception class for parameter loading errors
 */
class ParameterLoadException : public std::runtime_error
{
public:
  explicit ParameterLoadException(const std::string & message)
  : std::runtime_error(message) {}
};

/**
 * @brief Class for loading UBX-CFG parameters from JSON files
 * 
 * This class is responsible for loading parameter definitions from JSON files,
 * validating them, and providing access to the parameters. It also supports
 * filtering parameters by device type and firmware version.
 */
class UbxCfgParameterLoader
{
public:
  /**
   * @brief Constructor with parameter file path
   * @param file_path Path to the parameter file
   */
  explicit UbxCfgParameterLoader(const std::string & file_path);

  /**
   * @brief Load parameters from the file
   * @return True if parameters were loaded successfully
   * @throw ParameterLoadException if there was an error loading the parameters
   */
  bool load();

  /**
   * @brief Get parameter by name
   * @param name Parameter name
   * @return Parameter if found, nullopt otherwise
   */
  std::optional<UbxCfgParameter> get_parameter_by_name(const std::string & name) const;

  /**
   * @brief Get parameter by key ID
   * @param key_id Parameter key ID
   * @return Parameter if found, nullopt otherwise
   */
  std::optional<UbxCfgParameter> get_parameter_by_key_id(const ubx_key_id_t & key_id) const;

  /**
   * @brief Get all parameters
   * @return Vector of all parameters
   */
  std::vector<UbxCfgParameter> get_all_parameters() const;

  /**
   * @brief Get parameters for a specific device type
   * @param device_type Device type
   * @return Vector of parameters applicable to the device type
   */
  std::vector<UbxCfgParameter> get_parameters_for_device(const std::string & device_type) const;

  /**
   * @brief Get parameters for a specific device type and firmware version
   * @param device_type Device type
   * @param firmware_version Firmware version
   * @return Vector of parameters applicable to the device type and firmware version
   */
  std::vector<UbxCfgParameter> get_parameters_for_device_and_firmware(
    const std::string & device_type,
    const std::string & firmware_version) const;

  /**
   * @brief Get the available device types
   * @return Vector of available device types
   */
  std::vector<std::string> get_available_device_types() const;

  /**
   * @brief Get the available firmware versions for a device type
   * @param device_type Device type
   * @return Vector of available firmware versions
   */
  std::vector<std::string> get_available_firmware_versions(const std::string & device_type) const;

  /**
   * @brief Get the file path
   * @return File path
   */
  const std::string & get_file_path() const;

private:
  /**
   * @brief Parse a parameter from JSON
   * @param json_param JSON parameter object
   * @return Parsed parameter
   * @throw ParameterLoadException if there was an error parsing the parameter
   */
  UbxCfgParameter parse_parameter(const nlohmann::json & json_param);

  /**
   * @brief Parse firmware support information from JSON
   * @param json_support JSON firmware support object
   * @return Map of device type to firmware support information
   * @throw ParameterLoadException if there was an error parsing the firmware support
   */
  std::map<std::string, FirmwareSupport> parse_firmware_support(
    const nlohmann::json & json_support);

  /**
   * @brief Parse a behavior change from JSON
   * @param json_change JSON behavior change object
   * @return Parsed behavior change
   * @throw ParameterLoadException if there was an error parsing the behavior change
   */
  BehaviorChange parse_behavior_change(const nlohmann::json & json_change);

  /**
   * @brief Parse possible values from JSON
   * @param json_values JSON possible values object
   * @return Map of possible value names to values
   * @throw ParameterLoadException if there was an error parsing the possible values
   */
  std::map<std::string, std::string> parse_possible_values(const nlohmann::json & json_values);

  /**
   * @brief Parse a UBX type from string
   * @param type_str Type string
   * @return UBX type
   * @throw ParameterLoadException if the type is invalid
   */
  ubx_type_t parse_ubx_type(const std::string & type_str);

  /**
   * @brief Parse a UBX unit from string
   * @param unit_str Unit string
   * @return UBX unit
   * @throw ParameterLoadException if the unit is invalid
   */
  ubx_unit_t parse_ubx_unit(const std::string & unit_str);

  std::string file_path_;                                  ///< Path to the parameter file
  std::vector<UbxCfgParameter> parameters_;                ///< Vector of all parameters
  std::unordered_map<std::string, UbxCfgParameter> name_to_parameter_;  ///< Map of parameter names to parameters
  std::unordered_map<uint32_t, UbxCfgParameter> key_id_to_parameter_;  ///< Map of parameter key IDs to parameters
  std::vector<std::string> device_types_;                  ///< Vector of available device types
  std::map<std::string, std::vector<std::string>> firmware_versions_;  ///< Map of device types to firmware versions
};

}  // namespace ubx::cfg

#endif  // UBLOX_DGNSS_NODE__UBX__CFG__UBX_CFG_PARAMETER_LOADER_HPP_
