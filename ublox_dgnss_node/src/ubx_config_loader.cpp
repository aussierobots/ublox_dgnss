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
#include <fstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace
{
// Simple TOML parser for UBX config files
// Only handles: [section], key = "value", key = ["array", "items"]

struct TomlValue
{
  std::string string_value;
  std::vector<std::string> array_value;
  bool is_array = false;
};

using TomlSection = std::map<std::string, TomlValue>;
using TomlDocument = std::map<std::string, TomlSection>;

class SimpleTomlParser
{
public:
  static TomlDocument parse(const std::string & filepath)
  {
    std::ifstream file(filepath);
    if (!file.is_open()) {
      throw std::runtime_error("Cannot open TOML file: " + filepath);
    }

    TomlDocument doc;
    std::string current_section;
    std::string line;
    int line_num = 0;

    while (std::getline(file, line)) {
      line_num++;
      line = trim(line);

      // Skip empty lines and comments
      if (line.empty() || line[0] == '#') {continue;}

      // Parse section header: [section_name]
      if (line[0] == '[') {
        size_t end = line.find(']');
        if (end == std::string::npos) {
          throw std::runtime_error(
                  "Malformed section at line " + std::to_string(line_num) +
                  " in file: " + filepath);
        }
        current_section = trim(line.substr(1, end - 1));
        continue;
      }

      // Parse key = value
      size_t eq_pos = line.find('=');
      if (eq_pos == std::string::npos) {continue;}

      std::string key = trim(line.substr(0, eq_pos));
      std::string value_part = trim(line.substr(eq_pos + 1));

      if (current_section.empty()) {
        throw std::runtime_error(
                "Key '" + key + "' at line " + std::to_string(line_num) +
                " appears before any section in file: " + filepath);
      }

      TomlValue value;

      if (value_part[0] == '[') {
        // Array value
        value.is_array = true;
        value.array_value = parse_array(file, value_part, line_num, filepath);
      } else if (value_part[0] == '"') {
        // String value
        if (value_part.length() < 2 || value_part.back() != '"') {
          throw std::runtime_error(
                  "Unclosed string at line " + std::to_string(line_num) +
                  " in file: " + filepath);
        }
        value.string_value = value_part.substr(1, value_part.length() - 2);
      } else if (std::isdigit(value_part[0]) || value_part[0] == '-' || value_part[0] == '+') {
        // Number value - store as string for simplicity (we don't use these values)
        value.string_value = value_part;
      } else {
        throw std::runtime_error(
                "Unsupported value type at line " + std::to_string(line_num) +
                " in file: " + filepath);
      }

      doc[current_section][key] = value;
    }

    return doc;
  }

private:
  static std::string trim(const std::string & str)
  {
    size_t start = 0;
    size_t end = str.length();

    while (start < end && std::isspace(static_cast<unsigned char>(str[start]))) {
      start++;
    }
    while (end > start && std::isspace(static_cast<unsigned char>(str[end - 1]))) {
      end--;
    }

    return str.substr(start, end - start);
  }

  static std::vector<std::string> parse_array(
    std::ifstream & file,
    std::string current_line,
    int start_line_num,
    const std::string & filepath)
  {
    std::vector<std::string> result;
    std::string accumulated = current_line;

    // Read until we find closing ]
    while (accumulated.find(']') == std::string::npos) {
      std::string line;
      if (!std::getline(file, line)) {
        throw std::runtime_error(
                "Unclosed array starting at line " + std::to_string(start_line_num) +
                " in file: " + filepath);
      }
      accumulated += " " + trim(line);
    }

    // Extract content between [ and ]
    size_t start = accumulated.find('[');
    size_t end = accumulated.find(']');
    std::string content = accumulated.substr(start + 1, end - start - 1);

    // Parse quoted strings
    size_t pos = 0;
    while (pos < content.length()) {
      // Skip whitespace, commas, and newlines
      while (pos < content.length() &&
        (content[pos] == ' ' || content[pos] == ',' ||
        content[pos] == '\n' || content[pos] == '\t'))
      {
        pos++;
      }

      if (pos >= content.length()) {break;}

      // Parse quoted string
      if (content[pos] == '"') {
        size_t end_quote = content.find('"', pos + 1);
        if (end_quote == std::string::npos) {
          throw std::runtime_error(
                  "Unclosed quote in array starting at line " +
                  std::to_string(start_line_num) + " in file: " + filepath);
        }
        result.push_back(content.substr(pos + 1, end_quote - pos - 1));
        pos = end_quote + 1;
      } else {
        throw std::runtime_error(
                "Expected quoted string in array at line " +
                std::to_string(start_line_num) + " in file: " + filepath);
      }
    }

    return result;
  }
};

}  // anonymous namespace

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
    auto config = SimpleTomlParser::parse(toml_file_path);
    std::string family = config["metadata"]["device_family"].string_value;
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
    auto config = SimpleTomlParser::parse(toml_path);
    const auto & include_array = config["parameters"]["include"].array_value;

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
