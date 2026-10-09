// Copyright 2026 Australian Robotics Supplies & Technology
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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_DAHEADING_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_DAHEADING_HPP_

#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::daheading
{

struct flags_t
{
  union {
    u4_t all;
    struct
    {
      l_t gnss_fix_ok : 1;            // Bit 0
      l_t diff_soln : 1;              // Bit 1
      l_t rel_pos_valid : 1;          // Bit 2
      u1_t carr_soln : 2;             // Bits 3..4: 0 none, 1 float, 2 fixed
      u1_t reserved0 : 1;             // Bit 5
      l_t rel_pos_heading_valid : 1;  // Bit 6
      u4_t reserved1 : 25;            // Bits 7..31
    } bits;
  };
};

class NavDAHeadingPayload : public UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_DAHEADING;
  static constexpr u2_t PAYLOAD_LENGTH = 60;

  u1_t version = 0;         // Expected message version: 0x02
  u4_t itow = 0;            // ms - GPS time of week
  i4_t rel_pos_n = 0;       // mm - North component, antenna 1 to antenna 2
  i4_t rel_pos_e = 0;       // mm - East component
  i4_t rel_pos_d = 0;       // mm - Down component
  i4_t rel_pos_length = 0;  // mm - Baseline length
  // 1e-5 deg - Clockwise from True North; valid only when heading flag is set.
  // CFG-NAVSPG-DAHEADING_OFFSET affects only this field.
  i4_t rel_pos_heading = 0;
  u4_t acc_n = 0;           // mm
  u4_t acc_e = 0;           // mm
  u4_t acc_d = 0;           // mm
  u4_t acc_length = 0;      // mm
  u4_t acc_heading = 0;     // 1e-5 deg
  flags_t flags{};

public:
  NavDAHeadingPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  NavDAHeadingPayload(u1_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    if (payload_polled == nullptr || size != PAYLOAD_LENGTH) {
      throw std::invalid_argument("UBX-NAV-DAHEADING requires a 60-byte payload");
    }

    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    std::memcpy(payload_.data(), payload_polled, size);

    version = buf_offset<u1_t>(&payload_, 0);
    // Bytes 1..3 are reserved.
    itow = buf_offset<u4_t>(&payload_, 4);
    rel_pos_n = buf_offset<i4_t>(&payload_, 8);
    rel_pos_e = buf_offset<i4_t>(&payload_, 12);
    rel_pos_d = buf_offset<i4_t>(&payload_, 16);
    rel_pos_length = buf_offset<i4_t>(&payload_, 20);
    rel_pos_heading = buf_offset<i4_t>(&payload_, 24);
    // Bytes 28..31 are reserved.
    acc_n = buf_offset<u4_t>(&payload_, 32);
    acc_e = buf_offset<u4_t>(&payload_, 36);
    acc_d = buf_offset<u4_t>(&payload_, 40);
    acc_length = buf_offset<u4_t>(&payload_, 44);
    acc_heading = buf_offset<u4_t>(&payload_, 48);
    // Bytes 52..55 are reserved.
    flags.all = buf_offset<u4_t>(&payload_, 56);
  }

  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }

  std::string to_string()
  {
    std::ostringstream oss;
    oss << "version: " << static_cast<int>(version) << ", itow: " << itow;
    oss << ", rel_pos_n: " << rel_pos_n;
    oss << ", rel_pos_e: " << rel_pos_e;
    oss << ", rel_pos_d: " << rel_pos_d;
    oss << ", rel_pos_length: " << rel_pos_length;
    oss << ", rel_pos_heading: " << rel_pos_heading * 1e-5;
    oss << ", acc_n: " << acc_n;
    oss << ", acc_e: " << acc_e;
    oss << ", acc_d: " << acc_d;
    oss << ", acc_length: " << acc_length;
    oss << ", acc_heading: " << acc_heading * 1e-5;
    oss << ", flags: {";
    oss << "gnss_fix_ok: " << static_cast<int>(flags.bits.gnss_fix_ok);
    oss << ", diff_soln: " << static_cast<int>(flags.bits.diff_soln);
    oss << ", rel_pos_valid: " << static_cast<int>(flags.bits.rel_pos_valid);
    oss << ", carr_soln: " << static_cast<int>(flags.bits.carr_soln);
    oss << ", rel_pos_heading_valid: " << static_cast<int>(flags.bits.rel_pos_heading_valid);
    oss << "}";
    return oss.str();
  }
};

}  // namespace ubx::nav::daheading

#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_DAHEADING_HPP_
