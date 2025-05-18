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
 * @file ubx_cfg_parameter_loader.hpp
 * @brief Definition of the UbxCfgParameterLoader class for loading UBX-CFG parameters from TOML files
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

#include <toml.hpp>

namespace ubx::cfg
{

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
 * @brief Class for loading UBX-CFG parameters from TOML files
 *
 * This class is responsible for loading parameter definitions from TOML files,
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
   * @brief Check if a device type is valid/available
   * @param device_type Device type to check
   * @return True if the device type is valid, false otherwise
   */
  bool has_device_type(const std::string & device_type) const;

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
   * @brief Parse a parameter from TOML
   * @param toml_param TOML parameter object
   * @return Parsed parameter
   * @throw ParameterLoadException if there was an error parsing the parameter
   */
  UbxCfgParameter parse_parameter(const toml::value & toml_param);

  /**
   * @brief Parse firmware support information from TOML
   * @param toml_support TOML firmware support object
   * @return Map of device type to firmware support information
   * @throw ParameterLoadException if there was an error parsing the firmware support
   */
  std::map<std::string, FirmwareSupport> parse_firmware_support(
    const toml::value & toml_support);

  /**
   * @brief Parse a behavior change from TOML
   * @param toml_change TOML behavior change object
   * @return Parsed behavior change
   * @throw ParameterLoadException if there was an error parsing the behavior change
   */
  BehaviorChange parse_behavior_change(const toml::value & toml_change);

  /**
   * @brief Parse possible values from TOML
   * @param toml_values TOML possible values object
   * @return Map of possible value names to values
   * @throw ParameterLoadException if there was an error parsing the possible values
   */
  std::map<std::string, std::string> parse_possible_values(const toml::value & toml_values);

  /**
   * @brief Parse a UBX type string into a ubx_type_t enum value
   * @param type_str The type string to parse
   * @return The corresponding ubx_type_t value
   * @throws ParameterLoadException If the type string is invalid
   */
  ::ubx::ubx_type_t parse_ubx_type(const std::string & type_str);

  /**
   * @brief Parse a UBX unit string into a ubx_unit_t enum value
   * @param unit_str The unit string to parse
   * @return The corresponding ubx_unit_t value
   * @throws ParameterLoadException If the unit string is invalid
   */
  ::ubx::cfg::ubx_unit_t parse_ubx_unit(const std::string & unit_str);

  std::string file_path_;                                  ///< Path to the parameter file
  std::vector<UbxCfgParameter> parameters_;                ///< Vector of all parameters
  std::unordered_map<std::string, UbxCfgParameter> name_to_parameter_;  ///< Map of parameter names to parameters
  std::unordered_map<uint32_t, UbxCfgParameter> key_id_to_parameter_;  ///< Map of key IDs to parameters
  std::vector<std::string> device_types_;                  ///< Available device types
  std::map<std::string, std::vector<std::string>> firmware_versions_;  ///< Map of device types to firmware versions
};

}  // namespace ubx::cfg

#endif  // UBLOX_DGNSS_NODE__UBX__CFG__UBX_CFG_PARAMETER_LOADER_HPP_
