// Copyright 2021 Australian Robotics Supplies & Technology
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

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_CFG_ITEM_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_CFG_ITEM_HPP_
#include <unistd.h>
#include <map>
#include <list>
#include <string>
#include <sstream>
#include "ublox_dgnss_node/ubx/ubx_types.hpp"
#include "ublox_dgnss_node/ubx/ubx.hpp"

namespace ubx::cfg
{
size_t storage_size_bytes(u8_t storage_size_id)
{
  size_t size = 0;
  switch (storage_size_id) {
    case 0x01: size = 1; break;     /// one bit but uses one byte
    case 0x02: size = 1; break;
    case 0x03: size = 2; break;
    case 0x04: size = 4; break;
    case 0x05: size = 8; break;
    default:
      size = 0;
  }
  return size;
}

union ubx_key_id_t {
  u4_t all;
  struct  ubx_key_id_bit_t
  {
    u2_t item_id : 12;
    u1_t reserved2 : 4;
    u1_t group_id : 8;
    u1_t reserverd1 : 4;
    u1_t storage_size_id : 3;
    u1_t reserved0 : 1;
  } bits;
  u4_t key_id()
  {
    return all;
  }
  u1_t storage_size_id()
  {
    return bits.storage_size_id;
  }
  u1_t group_id()
  {
    return bits.group_id;
  }
  u2_t item_id()
  {
    return bits.item_id;
  }                                         // actually 12 bits
  size_t storage_size()
  {
    return storage_size_bytes(bits.storage_size_id);
  }
  std::string to_hex()
  {
    std::ostringstream oss;
    oss << "0x" << std::setfill('0') << std::setw(8) << std::right << std::hex << +all;
    return oss.str();
  }
};


struct key_value_t
{
  ubx_key_id_t ubx_key_id;
  value_t ubx_value;
};

enum ubx_unit_t
{
  NA,       // not applicable
  M,       // meters
  Y,       // years
  MONTH,       // months
  D,       // days
  H,       // hours
  MIN,       // minutes
  S,       // seconds
  HZ,       // frquency (Hz)
  MA,       // milli amps (mA)
  MS,       // milliseconds
  DEG,       // degree ie Longitude, Latitude
  MM,       // milimeter
  CM,       // centimeter
  MPS,       // meters per second
};

struct ubx_cfg_item_t
{
  const char * ubx_config_item;
  ubx_key_id_t ubx_key_id;
  ubx_type_t ubx_type;
  double_t scale;
  ubx_unit_t ubx_unit;
};
}  // namespace ubx::cfg

#endif  // UBLOX_DGNSS_NODE__UBX__UBX_CFG_ITEM_HPP_
