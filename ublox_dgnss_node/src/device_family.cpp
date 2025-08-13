// Copyright 2025 Australian Robotics Supplies & Technology
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

#include "ublox_dgnss_node/device_family.hpp"
#include <algorithm>

namespace ublox_dgnss
{

// Device family definitions with X20P, F9P and F9R support
const std::map<DeviceFamily, DeviceFamilyInfo> DEVICE_FAMILY_MAP = {
  {DeviceFamily::F9P, {
      "F9P",
      {0x01a9},  // Single product ID
      "F9P - High-precision GNSS",
      false,  // sensor_fusion_capable
      false,  // reliable_iserial (may need u-center programming)
      false  // dual_uart_capable
    }},
  {DeviceFamily::F9R, {
      "F9R",
      {0x01a9},  // Single product ID
      "F9R - High-precision GNSS with sensor fusion",
      true,  // sensor_fusion_capable (wheel ticks, ESF)
      false,  // reliable_iserial (may need u-center programming)
      false  // dual_uart_capable
    }},
  {DeviceFamily::X20P, {
      "X20P",
      {0x01ab, 0x050c, 0x050d},  // THREE separate USB devices: F9P-compatible + UART1 + UART2
      "X20P - All-band GNSS (multiple interfaces)",
      false,  // sensor_fusion_capable
      true,  // reliable_iserial (factory set for 0x050c/0x050d, user-programmed for 0x01ab)
      true   // dual_uart_capable
    }}
};

const std::map<std::string, DeviceFamily> DEVICE_FAMILY_LOOKUP = {
  {"F9P", DeviceFamily::F9P},
  {"F9R", DeviceFamily::F9R},
  {"X20P", DeviceFamily::X20P}
};

DeviceFamily string_to_device_family(const std::string & family_str)
{
  auto it = DEVICE_FAMILY_LOOKUP.find(family_str);
  return (it != DEVICE_FAMILY_LOOKUP.end()) ? it->second : DeviceFamily::F9P;
}

std::string device_family_to_string(DeviceFamily family)
{
  return DEVICE_FAMILY_MAP.at(family).name;
}

const DeviceFamilyInfo & get_device_family_info(DeviceFamily family)
{
  return DEVICE_FAMILY_MAP.at(family);
}

bool is_valid_device_family_string(const std::string & family_str)
{
  return DEVICE_FAMILY_LOOKUP.find(family_str) != DEVICE_FAMILY_LOOKUP.end();
}

bool DeviceFamilyInfo::has_product_id(uint16_t pid) const
{
  return std::find(product_ids.begin(), product_ids.end(), pid) != product_ids.end();
}

DeviceFamily find_device_family_by_product_id(uint16_t product_id)
{
  for (const auto & [family, info] : DEVICE_FAMILY_MAP) {
    if (info.has_product_id(product_id)) {
      return family;
    }
  }
  return DeviceFamily::F9P;  // Default fallback
}

}  // namespace ublox_dgnss
