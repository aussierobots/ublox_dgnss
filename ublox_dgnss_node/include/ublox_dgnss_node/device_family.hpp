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

#ifndef UBLOX_DGNSS_NODE__DEVICE_FAMILY_HPP_
#define UBLOX_DGNSS_NODE__DEVICE_FAMILY_HPP_

#include <string>
#include <map>
#include <vector>
#include <cstdint>

namespace ublox_dgnss
{

enum class DeviceFamily
{
  F9P,    // High-precision GNSS (product ID 0x01a9)
  F9R,    // High-precision GNSS with sensor fusion (product ID 0x01a9)
  X20P    // All-band GNSS with three USB interfaces (product IDs 0x01ab, 0x050c, 0x050d)
};

struct DeviceFamilyInfo
{
  std::string name;                    // Short name: "F9P", "F9R", "X20P"
  std::vector<uint16_t> product_ids;   // USB product IDs (multiple for X20P)
  std::string description;             // Full description for logging
  bool sensor_fusion_capable;          // F9R wheel tick and ESF support
  bool reliable_iserial;               // X20P has reliable iSerial, F9 may not
  bool dual_uart_capable;              // X20P has dual UART support

  // Helper method to get primary product ID (first in list)
  uint16_t primary_product_id() const {return product_ids.empty() ? 0 : product_ids[0];}

  // Helper method to check if a product ID belongs to this device family
  bool has_product_id(uint16_t pid) const;
};

// Device Family mapping
extern const std::map<DeviceFamily, DeviceFamilyInfo> DEVICE_FAMILY_MAP;

// String to DeviceFamily lookup
extern const std::map<std::string, DeviceFamily> DEVICE_FAMILY_LOOKUP;

// Helper functions
DeviceFamily string_to_device_family(const std::string & family_str);
std::string device_family_to_string(DeviceFamily family);
const DeviceFamilyInfo & get_device_family_info(DeviceFamily family);
bool is_valid_device_family_string(const std::string & family_str);
DeviceFamily find_device_family_by_product_id(uint16_t product_id);

}  // namespace ublox_dgnss

#endif  // UBLOX_DGNSS_NODE__DEVICE_FAMILY_HPP_
