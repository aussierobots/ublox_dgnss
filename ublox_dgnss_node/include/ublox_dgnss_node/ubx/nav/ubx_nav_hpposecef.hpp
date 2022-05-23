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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_HPPOSECEF_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_HPPOSECEF_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::hpposecef
{

struct invalid_ecef_t
{
  union {
    x1_t all;
    struct
    {
      l_t invalid_ecefX : 1;
      l_t invalid_ecefY : 1;
      l_t invalid_ecefZ : 1;
      l_t invalid_ecefXHp : 1;
      l_t invalid_ecefYHp : 1;
      l_t invalid_ecefZHp : 1;
    } bits;
  };
};

class NavHPPosECEFPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_HPPOSECEF;

  u1_t version;     // message version (0x00 for this version)
  u1_t reserved0[3];     // reserved
  u4_t iTOW;      // ms - GPS Time of week of the navigation epoch.
  i4_t ecefX;     // cm - ECEF X coordinate
  i4_t ecefY;     // cm - ECEF Y coordinate
  i4_t ecefZ;     // cm - ECEF Z coordinate
//  Must be in the range of -99..+99. Precise coordinate in cm = ecefX + (ecefXHp * 1e-2)
  i1_t ecefXHp;     // mm scale 0.1 - High Precision ECEF X coordinate
  i1_t ecefYHp;     // mm scale 0.1 - High Precision ECEF Y coordinate
  i1_t ecefZHp;     // mm scale 0.1 - High Precision ECEF Z coordinate
  invalid_ecef_t flags;     // 1 = Invalid ecefX, ecefY, ecefZ, ecefXHp, ecefYHp and ecefZHp
  u4_t pAcc;      // mm scale 0.1 - position accuracy estimate

public:
  NavHPPosECEFPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  NavHPPosECEFPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    version = payload_[0];
    iTOW = buf_offset<u4_t>(&payload_, 4);
    ecefX = buf_offset<i4_t>(&payload_, 8);
    ecefY = buf_offset<i4_t>(&payload_, 12);
    ecefZ = buf_offset<i4_t>(&payload_, 16);
    ecefXHp = buf_offset<i1_t>(&payload_, 20);
    ecefYHp = buf_offset<i1_t>(&payload_, 21);
    ecefZHp = buf_offset<i1_t>(&payload_, 22);
    flags.all = buf_offset<x1_t>(&payload_, 23);
    pAcc = buf_offset<u4_t>(&payload_, 24);
  }
  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }
  std::string to_string()
  {
    std::ostringstream oss;
    oss << "ver: " << +version;
    oss << " iTOW: " << +iTOW;
    oss << std::fixed;
    oss << " ecefX: " << +ecefX;
    oss << " ecefY: " << +ecefY;
    oss << " ecefZ: " << +ecefZ;
    oss << " ecefXHp: " << +ecefXHp;
    oss << " ecefYHp: " << +ecefYHp;
    oss << " ecefZHp: " << +ecefZHp;
    oss << " - flags invalid ecefX: " << +flags.bits.invalid_ecefX;
    oss << " ecefY: " << +flags.bits.invalid_ecefY;
    oss << " ecefZ: " << +flags.bits.invalid_ecefZ;
    oss << " ecefXHp: " << +flags.bits.invalid_ecefXHp;
    oss << " ecefYHp: " << +flags.bits.invalid_ecefYHp;
    oss << " ecefZHp: " << +flags.bits.invalid_ecefZHp;
    oss << "  - precise";
    oss << " ecefX: " << std::setprecision(3) << +(ecefX + (ecefXHp * 1e-2));
    oss << " ecefY: " << std::setprecision(3) << +(ecefY + (ecefYHp * 1e-2));
    oss << " ecefZ: " << std::setprecision(3) << +(ecefZ + (ecefZHp * 1e-2));
    oss << " pAcc: " << +pAcc * 0.1;

    return oss.str();
  }
};
}  // namespace ubx::nav::hpposecef
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_HPPOSECEF_HPP_
