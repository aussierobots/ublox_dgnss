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
 * @file ubx_cfg_parameter_loader.cpp
 * @brief Implementation of the UbxCfgParameterLoader class for loading UBX-CFG parameters from TOML files
 */

#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_parameter_loader.hpp"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <filesystem>

#include <toml.hpp>

namespace ubx::cfg
{

UbxCfgParameterLoader::UbxCfgParameterLoader(const std::string & file_path)
: file_path_(file_path)
{
}

bool UbxCfgParameterLoader::load()
{
  // Clear any existing data
  parameters_.clear();
  name_to_parameter_.clear();
  key_id_to_parameter_.clear();
  device_types_.clear();
  firmware_versions_.clear();

  try {
    // Check if the file exists
    if (!std::filesystem::exists(file_path_)) {
      throw ParameterLoadException("Parameter file not found: " + file_path_);
    }

    // Open the file
    std::ifstream file(file_path_);
    if (!file.is_open()) {
      throw ParameterLoadException("Failed to open parameter file: " + file_path_);
    }

    // Parse the TOML
    toml::value data = toml::parse(file_path_);

    // Check if the file has the required fields
    if (!data.contains("version") || !data.contains("device_types") || !data.contains("parameters")) {
      throw ParameterLoadException("Parameter file is missing required fields");
    }

    // Get the device types
    device_types_ = toml::get<std::vector<std::string>>(data.at("device_types"));

    // Get the firmware versions if available
    if (data.contains("firmware_versions")) {
      const auto& firmware_versions = toml::find(data, "firmware_versions");
      for (const auto& [device_type, versions_array] : firmware_versions.as_table()) {
        std::vector<std::string> version_list;
        for (const auto& version : versions_array.as_array()) {
          version_list.push_back(toml::find<std::string>(version, "version"));
        }
        firmware_versions_[device_type] = version_list;
      }
    }

    // Parse the parameters
    const auto& parameters = toml::find(data, "parameters").as_array();
    for (const auto& toml_param : parameters) {
      try {
        UbxCfgParameter param = parse_parameter(toml_param);
        parameters_.push_back(param);
        name_to_parameter_[param.get_name()] = param;
        key_id_to_parameter_[param.get_key_id().key_id()] = param;
      } catch (const ParameterLoadException & e) {
        // Log the error and continue with the next parameter
        std::cerr << "Error parsing parameter: " << e.what() << std::endl;
      }
    }

    return true;
  } catch (const toml::exception& e) {
    throw ParameterLoadException("TOML parsing error: " + std::string(e.what()));
  } catch (const std::exception & e) {
    throw ParameterLoadException("Error loading parameters: " + std::string(e.what()));
  }
}

std::optional<UbxCfgParameter> UbxCfgParameterLoader::get_parameter_by_name(
  const std::string & name) const
{
  auto it = name_to_parameter_.find(name);
  if (it != name_to_parameter_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::optional<UbxCfgParameter> UbxCfgParameterLoader::get_parameter_by_key_id(
  const ubx_key_id_t & key_id) const
{
  auto it = key_id_to_parameter_.find(key_id.key_id());
  if (it != key_id_to_parameter_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::vector<UbxCfgParameter> UbxCfgParameterLoader::get_all_parameters() const
{
  return parameters_;
}

std::vector<UbxCfgParameter> UbxCfgParameterLoader::get_parameters_for_device(
  const std::string & device_type) const
{
  std::vector<UbxCfgParameter> result;
  for (const auto & param : parameters_) {
    if (param.is_applicable_to_device(device_type)) {
      result.push_back(param);
    }
  }
  return result;
}

std::vector<UbxCfgParameter> UbxCfgParameterLoader::get_parameters_for_device_and_firmware(
  const std::string & device_type,
  const std::string & firmware_version) const
{
  std::vector<UbxCfgParameter> result;
  for (const auto & param : parameters_) {
    if (param.is_applicable_to_device(device_type) &&
      param.is_supported_in_firmware(device_type, firmware_version))
    {
      result.push_back(param);
    }
  }
  return result;
}

std::vector<std::string> UbxCfgParameterLoader::get_available_device_types() const
{
  return device_types_;
}

std::vector<std::string> UbxCfgParameterLoader::get_available_firmware_versions(
  const std::string & device_type) const
{
  auto it = firmware_versions_.find(device_type);
  if (it != firmware_versions_.end()) {
    return it->second;
  }
  return {};
}

const std::string & UbxCfgParameterLoader::get_file_path() const
{
  return file_path_;
}

UbxCfgParameter UbxCfgParameterLoader::parse_parameter(const toml::value & toml_param)
{
  // Check if the parameter has the required fields
  if (!toml_param.contains("name") || !toml_param.contains("key_id") ||
    !toml_param.contains("type") || !toml_param.contains("scale") ||
    !toml_param.contains("unit") || !toml_param.contains("applicable_devices") ||
    !toml_param.contains("description") || !toml_param.contains("group") ||
    !toml_param.contains("firmware_support"))
  {
    throw ParameterLoadException("Parameter is missing required fields");
  }

  try {
    // Extract parameter properties
    std::string name = toml::find<std::string>(toml_param, "name");
    std::string key_id_val = toml::find<std::string>(toml_param, "key_id");
    std::string type_str = toml::find<std::string>(toml_param, "type");
    double scale = toml::find<double>(toml_param, "scale");
    std::string unit_str = toml::find<std::string>(toml_param, "unit");
    std::vector<std::string> applicable_devices = toml::find<std::vector<std::string>>(toml_param, "applicable_devices");
    std::string description = toml::find<std::string>(toml_param, "description");
    std::string group = toml::find<std::string>(toml_param, "group");

    // Parse key_id
    ::ubx::cfg::ubx_key_id_t key_id;
    key_id.all = std::stoul(key_id_val, nullptr, 16);

    // Parse type
    ::ubx::ubx_type_t type = parse_ubx_type(type_str);

    // Parse unit
    ::ubx::cfg::ubx_unit_t unit = parse_ubx_unit(unit_str);

    // Parse firmware support
    auto firmware_support = parse_firmware_support(toml::find(toml_param, "firmware_support"));

    // Parse optional fields
    std::map<std::string, std::string> possible_values;
    if (toml_param.contains("possible_values")) {
      possible_values = parse_possible_values(toml::find(toml_param, "possible_values"));
    }

    std::string default_value = "";
    if (toml_param.contains("default_value")) {
      default_value = toml::find<std::string>(toml_param, "default_value");
    }

    std::optional<std::string> min_value = std::nullopt;
    if (toml_param.contains("min_value")) {
      min_value = toml::find<std::string>(toml_param, "min_value");
    }

    std::optional<std::string> max_value = std::nullopt;
    if (toml_param.contains("max_value")) {
      max_value = toml::find<std::string>(toml_param, "max_value");
    }

    // Create and return the parameter
    return UbxCfgParameter(
      name,
      key_id,
      type,
      scale,
      unit,
      applicable_devices,
      description,
      group,
      firmware_support,
      possible_values,
      default_value,
      min_value,
      max_value);
  } catch (const std::exception & e) {
    throw ParameterLoadException("Error parsing parameter: " + std::string(e.what()));
  }
}

std::map<std::string, FirmwareSupport> UbxCfgParameterLoader::parse_firmware_support(
  const toml::value & toml_support)
{
  std::map<std::string, FirmwareSupport> result;

  for (const auto & [device_type, support] : toml_support.as_table()) {
    FirmwareSupport firmware_support;

    // Check if the support has the required fields
    if (!support.contains("since")) {
      throw ParameterLoadException("Firmware support for " + device_type +
          " is missing 'since' field");
    }

    // Parse the support fields
    firmware_support.since = toml::find<std::string>(support, "since");
    if (support.contains("until")) {
      firmware_support.until = toml::find<std::string>(support, "until");
    }

    // Parse behavior changes
    if (support.contains("behavior_changes")) {
      const auto& behavior_changes = toml::find(support, "behavior_changes").as_array();
      for (const auto & toml_change : behavior_changes) {
        firmware_support.behavior_changes.push_back(parse_behavior_change(toml_change));
      }
    }

    result[device_type] = firmware_support;
  }

  return result;
}

BehaviorChange UbxCfgParameterLoader::parse_behavior_change(const toml::value & toml_change)
{
  // Check if the behavior change has the required fields
  if (!toml_change.contains("version") || !toml_change.contains("description")) {
    throw ParameterLoadException("Behavior change is missing required fields");
  }

  // Parse the behavior change fields
  BehaviorChange change;
  change.version = toml::find<std::string>(toml_change, "version");
  change.description = toml::find<std::string>(toml_change, "description");

  return change;
}

std::map<std::string, std::string> UbxCfgParameterLoader::parse_possible_values(
  const toml::value & toml_values)
{
  std::map<std::string, std::string> result;

  for (const auto & [name, value] : toml_values.as_table()) {
    result[name] = toml::get<std::string>(value);
  }

  return result;
}

::ubx::ubx_type_t UbxCfgParameterLoader::parse_ubx_type(const std::string & type_str)
{
  if (type_str == "L") {return ::ubx::L;} else if (type_str == "U1") {
    return ::ubx::U1;
  } else if (type_str == "I1") {return ::ubx::I1;} else if (type_str == "E1") {
    return ::ubx::E1;
  } else if (type_str == "X1") {return ::ubx::X1;} else if (type_str == "U2") {
    return ::ubx::U2;
  } else if (type_str == "I2") {return ::ubx::I2;} else if (type_str == "E2") {
    return ::ubx::E2;
  } else if (type_str == "X2") {return ::ubx::X2;} else if (type_str == "U4") {
    return ::ubx::U4;
  } else if (type_str == "I4") {return ::ubx::I4;} else if (type_str == "E4") {
    return ::ubx::E4;
  } else if (type_str == "X4") {return ::ubx::X4;} else if (type_str == "R4") {
    return ::ubx::R4;
  } else if (type_str == "U8") {return ::ubx::U8;} else if (type_str == "I8") {
    return ::ubx::I8;
  } else if (type_str == "X8") {return ::ubx::X8;} else if (type_str == "R8") {
    return ::ubx::R8;
  } else {
    throw ParameterLoadException("Invalid UBX type: " + type_str);
  }
}

::ubx::cfg::ubx_unit_t UbxCfgParameterLoader::parse_ubx_unit(const std::string & unit_str)
{
  if (unit_str == "NA") {return ::ubx::cfg::NA;} else if (unit_str == "M") {
    return ::ubx::cfg::M;
  } else if (unit_str == "Y") {return ::ubx::cfg::Y;} else if (unit_str == "MONTH") {
    return ::ubx::cfg::MONTH;
  } else if (unit_str == "D") {return ::ubx::cfg::D;} else if (unit_str == "H") {
    return ::ubx::cfg::H;
  } else if (unit_str == "MIN") {return ::ubx::cfg::MIN;} else if (unit_str == "S") {
    return ::ubx::cfg::S;
  } else if (unit_str == "HZ") {return ::ubx::cfg::HZ;} else if (unit_str == "MA") {
    return ::ubx::cfg::MA;
  } else if (unit_str == "MS") {return ::ubx::cfg::MS;} else if (unit_str == "DEG") {
    return ::ubx::cfg::DEG;
  } else if (unit_str == "MM") {return ::ubx::cfg::MM;} else if (unit_str == "CM") {
    return ::ubx::cfg::CM;
  } else if (unit_str == "MPS") {return ::ubx::cfg::MPS;} else {
    throw ParameterLoadException("Invalid UBX unit: " + unit_str);
  }
}

}  // namespace ubx::cfg
