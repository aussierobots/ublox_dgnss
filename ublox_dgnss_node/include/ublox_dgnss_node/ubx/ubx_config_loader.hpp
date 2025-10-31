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

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_CONFIG_LOADER_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_CONFIG_LOADER_HPP_

#include <string>
#include <set>
#include <map>
#include "ublox_dgnss_node/ubx/ubx_cfg_item.hpp"
#include "ublox_dgnss_node/ubx/ubx_cfg_item_map.hpp"

namespace ubx
{
namespace cfg
{

class UbxConfigLoader
{
public:
  // Load filtered map from TOML file
  static ubx_cfg_item_map_t load_from_toml(
    const std::string & toml_file_path,
    const ubx_cfg_item_map_t & source_map);

  // Get default TOML path for device family
  static std::string get_default_toml_path(const std::string & device_family);

  // Extract device family from TOML metadata
  static std::string get_toml_device_family(const std::string & toml_file_path);

  // Normalize device family string (case-insensitive)
  static std::string normalize_device_family(const std::string & family);

private:
  // Parse TOML include list
  static std::set<std::string> parse_include_list(const std::string & toml_path);

  // Validate that all TOML parameters exist in source map
  static void validate_parameters(
    const std::set<std::string> & include_list,
    const ubx_cfg_item_map_t & source_map,
    const std::string & toml_path);
};

}  // namespace cfg
}  // namespace ubx

#endif  // UBLOX_DGNSS_NODE__UBX__UBX_CONFIG_LOADER_HPP_
