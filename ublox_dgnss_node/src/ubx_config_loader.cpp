// Copyright (c) 2025 Australian Robotics Supplies & Technology
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

#include "ublox_dgnss_node/ubx/ubx_config_loader.hpp"
#include "ublox_dgnss_node/ubx/ubx_cfg_item_map.hpp"

#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <vector>

#include <toml.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ubx
{
namespace cfg
{

ubx_cfg_item_map_t UbxConfigLoader::load_from_toml(
  const std::string & toml_file_path,
  const ubx_cfg_item_map_t & source_map)
{
  // Parse include list from TOML
  std::set<std::string> include_list = parse_include_list(toml_file_path);

  // Validate all parameters exist in source map
  validate_parameters(include_list, source_map, toml_file_path);

  // Build name-to-keyid lookup map
  std::map<std::string, ubx_key_id_t> name_to_keyid;
  for (const auto & [key_id, cfg_item] : source_map) {
    name_to_keyid[cfg_item.ubx_config_item] = key_id;
  }

  // Create filtered map (shallow copy - reuses static string pointers)
  ubx_cfg_item_map_t filtered_map;
  for (const auto & param_name : include_list) {
    ubx_key_id_t key_id = name_to_keyid.at(param_name);
    filtered_map[key_id] = source_map.at(key_id);
  }

  // Sanity check - ensure we have a reasonable number of parameters
  if (filtered_map.size() < 10) {
    throw std::runtime_error(
            "TOML config resulted in very small map (" +
            std::to_string(filtered_map.size()) + " parameters)"
    );
  }

  return filtered_map;
}

std::string UbxConfigLoader::get_default_toml_path(const std::string & device_family)
{
  std::string package_share =
    ament_index_cpp::get_package_share_directory("ublox_dgnss");

  std::string family_lower = device_family;
  std::transform(
    family_lower.begin(), family_lower.end(),
    family_lower.begin(), ::tolower);

  return package_share + "/config/" + family_lower + "_ubx_config.toml";
}

std::string UbxConfigLoader::get_toml_device_family(const std::string & toml_file_path)
{
  try {
    const auto config = toml::parse(toml_file_path);
    const auto metadata = toml::find(config, "metadata");
    std::string family = toml::find<std::string>(metadata, "device_family");
    return normalize_device_family(family);
  } catch (const std::exception & e) {
    return "UNKNOWN";
  }
}

std::string UbxConfigLoader::normalize_device_family(const std::string & family)
{
  std::string normalized = family;
  std::transform(
    normalized.begin(), normalized.end(),
    normalized.begin(), ::toupper);
  return normalized;
}

std::set<std::string> UbxConfigLoader::parse_include_list(const std::string & toml_path)
{
  std::set<std::string> result;

  try {
    const auto config = toml::parse(toml_path);
    const auto params = toml::find(config, "parameters");
    const auto & include_array = toml::find<std::vector<std::string>>(params, "include");

    for (const auto & name : include_array) {
      result.insert(name);
    }
  } catch (const std::exception & e) {
    throw std::runtime_error(
            "Failed to parse TOML file '" + toml_path + "': " + std::string(e.what())
    );
  }

  return result;
}

void UbxConfigLoader::validate_parameters(
  const std::set<std::string> & include_list,
  const ubx_cfg_item_map_t & source_map,
  const std::string & toml_path)
{
  // Build available parameter names
  std::set<std::string> available_params;
  for (const auto & [key_id, cfg_item] : source_map) {
    available_params.insert(cfg_item.ubx_config_item);
  }

  // Find missing parameters
  std::vector<std::string> missing;
  for (const auto & param_name : include_list) {
    if (available_params.find(param_name) == available_params.end()) {
      missing.push_back(param_name);
    }
  }

  // Report errors if any parameters are missing
  if (!missing.empty()) {
    std::ostringstream oss;
    oss << "TOML file '" << toml_path << "' references unknown parameters:\n";
    for (size_t i = 0; i < missing.size() && i < 10; ++i) {
      oss << "  - " << missing[i] << "\n";
    }
    if (missing.size() > 10) {
      oss << "  ... and " << (missing.size() - 10) << " more\n";
    }
    throw std::runtime_error(oss.str());
  }
}

}  // namespace cfg
}  // namespace ubx
