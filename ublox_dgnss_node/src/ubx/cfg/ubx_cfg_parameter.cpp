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
 * @file ubx_cfg_parameter.cpp
 * @brief Implementation of the UbxCfgParameter class for handling UBX-CFG parameters
 */

#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_parameter.hpp"

#include <algorithm>
#include <regex>
#include <sstream>
#include <iomanip>

namespace ubx::cfg {

UbxCfgParameter::UbxCfgParameter(
  const std::string & name,
  const ::ubx::cfg::ubx_key_id_t & key_id,
  const ::ubx::ubx_type_t & type,
  double scale,
  const ::ubx::cfg::ubx_unit_t & unit,
  const std::vector<std::string> & applicable_devices,
  const std::string & description,
  const std::string & group,
  const std::map<std::string, FirmwareSupport> & firmware_support,
  const std::map<std::string, std::string> & possible_values,
  const std::string & default_value,
  const std::optional<std::string> & min_value,
  const std::optional<std::string> & max_value)
: name_(name),
  key_id_(key_id),
  type_(type),
  scale_(scale),
  unit_(unit),
  applicable_devices_(applicable_devices),
  description_(description),
  group_(group),
  firmware_support_(firmware_support),
  possible_values_(possible_values),
  default_value_(default_value),
  min_value_(min_value),
  max_value_(max_value)
{
  // Ensure scale is greater than 0
  if (scale_ <= 0.0) {
    scale_ = 1.0;
  }
}

const std::string & UbxCfgParameter::get_name() const
{
  return name_;
}

::ubx::cfg::ubx_key_id_t UbxCfgParameter::get_key_id() const
{
  return key_id_;
}

::ubx::ubx_type_t UbxCfgParameter::get_type() const
{
  return type_;
}

double UbxCfgParameter::get_scale() const
{
  return scale_;
}

::ubx::cfg::ubx_unit_t UbxCfgParameter::get_unit() const
{
  return unit_;
}

const std::vector<std::string> & UbxCfgParameter::get_applicable_devices() const
{
  return applicable_devices_;
}

const std::string & UbxCfgParameter::get_description() const
{
  return description_;
}

const std::string & UbxCfgParameter::get_group() const
{
  return group_;
}

const std::map<std::string, FirmwareSupport> & UbxCfgParameter::get_firmware_support() const
{
  return firmware_support_;
}

const std::map<std::string, std::string> & UbxCfgParameter::get_possible_values() const
{
  return possible_values_;
}

const std::string & UbxCfgParameter::get_default_value() const
{
  return default_value_;
}

const std::optional<std::string> & UbxCfgParameter::get_min_value() const
{
  return min_value_;
}

const std::optional<std::string> & UbxCfgParameter::get_max_value() const
{
  return max_value_;
}

bool UbxCfgParameter::is_applicable_to_device(const std::string & device_type) const
{
  return std::find(
    applicable_devices_.begin(),
    applicable_devices_.end(),
    device_type) != applicable_devices_.end();
}

bool UbxCfgParameter::is_supported_in_firmware(
  const std::string & device_type,
  const std::string & firmware_version) const
{
  // Check if the parameter is applicable to the device type
  if (!is_applicable_to_device(device_type)) {
    return false;
  }

  // Check if the parameter has firmware support information for this device
  auto it = firmware_support_.find(device_type);
  if (it == firmware_support_.end()) {
    return false;
  }

  const auto & support = it->second;

  // Check if the firmware version is supported
  // Parameter must have been introduced in this or earlier firmware
  if (compare_firmware_versions(firmware_version, support.since) < 0) {
    return false;
  }

  // If parameter has been deprecated, check if current firmware is still supported
  if (support.until.has_value() &&
    compare_firmware_versions(firmware_version, support.until.value()) >= 0)
  {
    return false;
  }

  return true;
}

std::string UbxCfgParameter::get_behavior_change(
  const std::string & device_type,
  const std::string & firmware_version) const
{
  // Check if the parameter is applicable to the device type
  if (!is_applicable_to_device(device_type)) {
    return "";
  }

  // Check if the parameter has firmware support information for this device
  auto it = firmware_support_.find(device_type);
  if (it == firmware_support_.end()) {
    return "";
  }

  const auto & support = it->second;

  // Check if there are any behavior changes for this firmware version
  for (const auto & change : support.behavior_changes) {
    if (compare_firmware_versions(firmware_version, change.version) >= 0) {
      return change.description;
    }
  }

  return "";
}

value_t UbxCfgParameter::string_to_ubx_value(const std::string & value_str) const
{
  value_t value;

  // For enum types, check if the value is a named value
  if (type_ == ubx_type_t::E1 && !possible_values_.empty()) {
    // Check if the value is a named value
    for (const auto & [name, val] : possible_values_) {
      if (value_str == name) {
        // Convert hex string to integer
        uint64_t int_val = parse_hex_string(val);
        value.u1 = static_cast<uint8_t>(int_val);
        return value;
      }
    }

    // If not a named value, try to parse as hex
    try {
      uint64_t int_val = parse_hex_string(value_str);
      value.u1 = static_cast<uint8_t>(int_val);
      return value;
    } catch (const std::invalid_argument &) {
      throw std::invalid_argument("Invalid enum value: " + value_str);
    }
  }

  // Handle other types based on the UBX type
  switch (type_) {
    case ubx_type_t::L:
      value.l = (value_str == "true" || value_str == "1");
      break;
    case ubx_type_t::U1:
      try {
        if (value_str.substr(0, 2) == "0x") {
          value.u1 = static_cast<uint8_t>(parse_hex_string(value_str));
        } else {
          value.u1 = static_cast<uint8_t>(std::stoul(value_str));
        }
      } catch (const std::exception &) {
        throw std::invalid_argument("Invalid U1 value: " + value_str);
      }
      break;
    case ubx_type_t::U2:
      try {
        if (value_str.substr(0, 2) == "0x") {
          value.u2 = static_cast<uint16_t>(parse_hex_string(value_str));
        } else {
          value.u2 = static_cast<uint16_t>(std::stoul(value_str));
        }
      } catch (const std::exception &) {
        throw std::invalid_argument("Invalid U2 value: " + value_str);
      }
      break;
    case ubx_type_t::U4:
      try {
        if (value_str.substr(0, 2) == "0x") {
          value.u4 = static_cast<uint32_t>(parse_hex_string(value_str));
        } else {
          value.u4 = static_cast<uint32_t>(std::stoul(value_str));
        }
      } catch (const std::exception &) {
        throw std::invalid_argument("Invalid U4 value: " + value_str);
      }
      break;
    case ubx_type_t::I1:
      try {
        value.i1 = static_cast<int8_t>(std::stoi(value_str));
      } catch (const std::exception &) {
        throw std::invalid_argument("Invalid I1 value: " + value_str);
      }
      break;
    case ubx_type_t::I2:
      try {
        value.i2 = static_cast<int16_t>(std::stoi(value_str));
      } catch (const std::exception &) {
        throw std::invalid_argument("Invalid I2 value: " + value_str);
      }
      break;
    case ubx_type_t::I4:
      try {
        value.i4 = static_cast<int32_t>(std::stol(value_str));
      } catch (const std::exception &) {
        throw std::invalid_argument("Invalid I4 value: " + value_str);
      }
      break;
    case ubx_type_t::R4:
      try {
        value.r4 = static_cast<float>(std::stof(value_str));
      } catch (const std::exception &) {
        throw std::invalid_argument("Invalid R4 value: " + value_str);
      }
      break;
    case ubx_type_t::R8:
      try {
        value.r8 = static_cast<double>(std::stod(value_str));
      } catch (const std::exception &) {
        throw std::invalid_argument("Invalid R8 value: " + value_str);
      }
      break;
    case ubx_type_t::X1:
    case ubx_type_t::X2:
    case ubx_type_t::X4:
      try {
        uint64_t int_val = parse_hex_string(value_str);
        if (type_ == ubx_type_t::X1) {
          value.x1 = static_cast<uint8_t>(int_val);
        } else if (type_ == ubx_type_t::X2) {
          value.x2 = static_cast<uint16_t>(int_val);
        } else {  // X4
          value.x4 = static_cast<uint32_t>(int_val);
        }
      } catch (const std::exception &) {
        throw std::invalid_argument("Invalid X value: " + value_str);
      }
      break;
    default:
      throw std::invalid_argument("Unsupported type: " + std::to_string(static_cast<int>(type_)));
  }

  return value;
}

std::string UbxCfgParameter::ubx_value_to_string(const value_t & value) const
{
  std::stringstream ss;

  // For enum types, check if the value is a named value
  if (type_ == ubx_type_t::E1 && !possible_values_.empty()) {
    // Convert value to hex string
    ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(value.u1);
    std::string hex_value = ss.str();

    // Check if the value is a named value
    for (const auto & [name, val] : possible_values_) {
      if (val == hex_value) {
        return name;
      }
    }

    // If not a named value, return the hex value
    return hex_value;
  }

  // Handle other types based on the UBX type
  switch (type_) {
    case ubx_type_t::L:
      return value.l ? "true" : "false";
    case ubx_type_t::U1:
      return std::to_string(value.u1);
    case ubx_type_t::U2:
      return std::to_string(value.u2);
    case ubx_type_t::U4:
      return std::to_string(value.u4);
    case ubx_type_t::I1:
      return std::to_string(value.i1);
    case ubx_type_t::I2:
      return std::to_string(value.i2);
    case ubx_type_t::I4:
      return std::to_string(value.i4);
    case ubx_type_t::R4:
      return std::to_string(value.r4);
    case ubx_type_t::R8:
      return std::to_string(value.r8);
    case ubx_type_t::X1:
      ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(value.x1);
      return ss.str();
    case ubx_type_t::X2:
      ss << "0x" << std::hex << std::setw(4) << std::setfill('0') << value.x2;
      return ss.str();
    case ubx_type_t::X4:
      ss << "0x" << std::hex << std::setw(8) << std::setfill('0') << value.x4;
      return ss.str();
    default:
      throw std::invalid_argument("Unsupported type: " + std::to_string(static_cast<int>(type_)));
  }
}

bool UbxCfgParameter::validate_string_value(const std::string & value_str) const
{
  try {
    // Convert the string value to a UBX value
    value_t value = string_to_ubx_value(value_str);

    // Validate the UBX value
    return validate_ubx_value(value);
  } catch (const std::exception &) {
    return false;
  }
}

bool UbxCfgParameter::validate_ubx_value(const value_t & value) const
{
  // For enum types, check if the value is a valid enum value
  if (type_ == ubx_type_t::E1 && !possible_values_.empty()) {
    // Convert value to hex string
    std::stringstream ss;
    ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(value.u1);
    std::string hex_value = ss.str();

    // Check if the value is a valid enum value
    for (const auto & [name, val] : possible_values_) {
      if (val == hex_value) {
        return true;
      }
    }

    // If not in possible_values, check if it's within the min/max range
    if (max_value_.has_value()) {
      value_t max_val = string_to_ubx_value(max_value_.value());
      if (value.u1 <= max_val.u1) {
        return true;
      }
    }

    return false;
  }

  // For numeric types, check if the value is within the min/max range
  if (min_value_.has_value() || max_value_.has_value()) {
    // Convert the value to a string
    std::string value_str = ubx_value_to_string(value);

    // Convert the min/max values to UBX values
    if (min_value_.has_value()) {
      value_t min_val = string_to_ubx_value(min_value_.value());
      // Compare the values based on the type
      switch (type_) {
        case ubx_type_t::U1:
          if (value.u1 < min_val.u1) {return false;}
          break;
        case ubx_type_t::U2:
          if (value.u2 < min_val.u2) {return false;}
          break;
        case ubx_type_t::U4:
          if (value.u4 < min_val.u4) {return false;}
          break;
        case ubx_type_t::I1:
          if (value.i1 < min_val.i1) {return false;}
          break;
        case ubx_type_t::I2:
          if (value.i2 < min_val.i2) {return false;}
          break;
        case ubx_type_t::I4:
          if (value.i4 < min_val.i4) {return false;}
          break;
        case ubx_type_t::R4:
          if (value.r4 < min_val.r4) {return false;}
          break;
        case ubx_type_t::R8:
          if (value.r8 < min_val.r8) {return false;}
          break;
        default:
          break;
      }
    }

    if (max_value_.has_value()) {
      value_t max_val = string_to_ubx_value(max_value_.value());
      // Compare the values based on the type
      switch (type_) {
        case ubx_type_t::U1:
          if (value.u1 > max_val.u1) {return false;}
          break;
        case ubx_type_t::U2:
          if (value.u2 > max_val.u2) {return false;}
          break;
        case ubx_type_t::U4:
          if (value.u4 > max_val.u4) {return false;}
          break;
        case ubx_type_t::I1:
          if (value.i1 > max_val.i1) {return false;}
          break;
        case ubx_type_t::I2:
          if (value.i2 > max_val.i2) {return false;}
          break;
        case ubx_type_t::I4:
          if (value.i4 > max_val.i4) {return false;}
          break;
        case ubx_type_t::R4:
          if (value.r4 > max_val.r4) {return false;}
          break;
        case ubx_type_t::R8:
          if (value.r8 > max_val.r8) {return false;}
          break;
        default:
          break;
      }
    }
  }

  return true;
}

int UbxCfgParameter::compare_firmware_versions(
  const std::string & version1,
  const std::string & version2)
{
  // Extract firmware type and version number using regex
  // Format is typically "HPG 1.13" or "HPS 1.30"
  std::regex pattern("([A-Z]+)\\s+(\\d+)\\.(\\d+)");
  std::smatch matches1, matches2;

  if (!std::regex_search(version1, matches1, pattern) ||
    !std::regex_search(version2, matches2, pattern))
  {
    // If either version doesn't match the expected format, compare as strings
    return version1.compare(version2);
  }

  // Compare firmware types
  std::string type1 = matches1[1].str();
  std::string type2 = matches2[1].str();
  if (type1 != type2) {
    return type1.compare(type2);
  }

  // Compare major version
  int major1 = std::stoi(matches1[2].str());
  int major2 = std::stoi(matches2[2].str());
  if (major1 != major2) {
    return major1 - major2;
  }

  // Compare minor version
  int minor1 = std::stoi(matches1[3].str());
  int minor2 = std::stoi(matches2[3].str());
  return minor1 - minor2;
}

uint64_t UbxCfgParameter::parse_hex_string(const std::string & hex_str)
{
  // Check if the string is a valid hex number
  if (hex_str.substr(0, 2) != "0x") {
    throw std::invalid_argument("Invalid hex string: " + hex_str);
  }

  // Parse the hex string
  try {
    return std::stoull(hex_str, nullptr, 16);
  } catch (const std::exception &) {
    throw std::invalid_argument("Invalid hex string: " + hex_str);
  }
}

}  // namespace ubx::cfg
