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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_SVIN_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_SVIN_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::svin
{
class NavSvinPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_SVIN;

  u1_t version;     // message version (0x00 for this version)
  u4_t iTOW;     // message version (0x00 for this version)
  u4_t dur;     // message version (0x00 for this version)
  i4_t meanX;     // message version (0x00 for this version)
  i4_t meanY;     // message version (0x00 for this version)
  i4_t meanZ;     // message version (0x00 for this version)
  i1_t meanXHP;     // message version (0x00 for this version)
  i1_t meanYHP;     // message version (0x00 for this version)
  i1_t meanZHP;     // message version (0x00 for this version)
  u4_t meanAcc;     // message version (0x00 for this version)
  u4_t obs;     // message version (0x00 for this version)
  u1_t valid;     // message version (0x00 for this version)
  u1_t active;     // message version (0x00 for this version)

public:
  NavSvinPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  NavSvinPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    version = payload_[0];
    iTOW = buf_offset<u4_t>(&payload_, 4);
    dur = buf_offset<u4_t>(&payload_, 8);
    meanX = buf_offset<i4_t>(&payload_, 12);
    meanY = buf_offset<i4_t>(&payload_, 16);
    meanZ = buf_offset<i4_t>(&payload_, 20);
    meanXHP = buf_offset<i1_t>(&payload_, 24);
    meanYHP = buf_offset<i1_t>(&payload_, 25);
    meanZHP = buf_offset<i1_t>(&payload_, 26);
    meanAcc = buf_offset<u4_t>(&payload_, 28);
    obs = buf_offset<u4_t>(&payload_, 32);
    valid = buf_offset<u1_t>(&payload_, 36);
    active = buf_offset<u1_t>(&payload_, 37);
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
    oss << " dur: " << +dur;
    oss << " meanX: " << +meanX;
    oss << " meanY: " << +meanY;
    oss << " meanZ: " << +meanZ;
    oss << " meanXHP: " << +meanXHP;
    oss << " meanYHP: " << +meanYHP;
    oss << " meanZHP: " << +meanZHP;
    oss << " meanAcc: " << +meanAcc;
    oss << " obs: " << +obs;
    oss << " valid: " << +valid;
    oss << " active: " << +active;

    return oss.str();
  }
};
}  // namespace ubx::nav::svin
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_SVIN_HPP_
