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

#ifndef UBLOX_DGNSS_NODE__UBX__CFG__UBX_CFG_ITEM_HPP_
#define UBLOX_DGNSS_NODE__UBX__CFG__UBX_CFG_ITEM_HPP_
#include <unistd.h>
#include <map>
#include <list>
#include <string>
#include <sstream>
#include "ublox_dgnss_node/ubx/ubx_types.hpp"
#include "ublox_dgnss_node/ubx/ubx.hpp"

namespace ubx::cfg
{
/**
 * @brief Get the storage size in bytes for a given storage size ID
 * @param storage_size_id The storage size ID
 * @return The storage size in bytes
 */
size_t storage_size_bytes(u8_t storage_size_id);

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
  u4_t key_id() const
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
  ubx::value_t ubx_value;
};

enum ubx_unit_t
{
  NA,       // not applicable
  M,       // meters
  Y,       // years
  MONTH,   // months
  D,       // days
  H,       // hours
  MIN,     // minutes
  S,       // seconds
  MS,      // milliseconds
  HZ,      // frequency (hertz, cycles per second)
  MA,      // milliamperes
  DEG,     // degrees
  PCT,     // percentage
  BAUD,    // baud rate (bits per second)
  KBPS,    // kilobits per second
  PC,      // parts per century 0-100, percentage by 100 (0-100 pct)
  DB,      // decibel
  DBM,     // decibel-milliwatt
  MDEG,    // millidegree
  MHZ,     // megahertz
};


// The RXM class ID is 0x02 as per u-blox protocol
enum ubx_type_t
{
  L = 0,            // logical bit mask
  E1 = 1,           // single byte enumeration
  E2 = 2,           // two byte enumeration
  E4 = 3,           // four byte enumeration
  U1 = 4,           // single byte unsigned integer
  U2 = 5,           // two byte unsgined integer
  U4 = 6,           // four byte unsigned integer
  U8 = 7,           // eight byte unsigned integer
  I1 = 8,           // single byte signed integer
  I2 = 9,           // two byte signed integer
  I4 = 10,          // four byte signed integer
  I8 = 11,          // eight byte signed integer
  X1 = 12,          // single byte hex string
  X2 = 13,          // two byte hex string
  X4 = 14,          // four byte hex string
  X8 = 15,          // eight byte hex string
  R4 = 16,          // single precision floating point
  R8 = 17,          // double precision floating point
};

struct ubx_cfg_item_t
{
  std::string name;
  uint32_t key_id;
  ubx_type_t type;
  double scale;
  ubx_unit_t unit;
};

/**
 * @brief Map of UBX key IDs to UBX configuration items
 */
extern std::map<ubx_key_id_t, ubx_cfg_item_t> ubxKeyCfgItemMap;

/**
 * @brief Compare two UBX key IDs
 * @param fk1 First key ID
 * @param fk2 Second key ID
 * @return True if fk1 is less than fk2
 */
bool operator<(const ubx_key_id_t & fk1, const ubx_key_id_t & fk2);

/**
 * @brief Compare two UBX key IDs for equality
 * @param fk1 First key ID
 * @param fk2 Second key ID
 * @return True if the key IDs are equal
 */
bool operator==(const ubx_key_id_t & fk1, const ubx_key_id_t & fk2);

/**
 * @brief Compare a UBX key ID with an integer
 * @param fk1 UBX key ID
 * @param value Integer value
 * @return True if the key ID equals the integer value
 */
bool operator==(const ubx_key_id_t & fk1, uint32_t value);

/**
 * @brief Compare an integer with a UBX key ID
 * @param value Integer value
 * @param fk1 UBX key ID
 * @return True if the integer value equals the key ID
 */
bool operator==(uint32_t value, const ubx_key_id_t & fk1);
}  // namespace ubx::cfg

#endif  // UBLOX_DGNSS_NODE__UBX__CFG__UBX_CFG_ITEM_HPP_