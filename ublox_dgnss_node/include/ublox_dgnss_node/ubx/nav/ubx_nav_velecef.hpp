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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_VELECEF_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_VELECEF_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::velecef
{

class NavVelECEFPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_VELECEF;

  u4_t iTOW;      // ms - GPS Time of week of the navigation epoch.
  i4_t ecefVX;     // cm/s - ECEF X velocity
  i4_t ecefVY;     // cm/s - ECEF Y velocity
  i4_t ecefVZ;     // cm/s - ECEF Z velocity
  u4_t sAcc;      // cm/s - velocity accuracy estimate

public:
  NavVelECEFPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  NavVelECEFPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    iTOW = buf_offset<u4_t>(&payload_, 0);
    ecefVX = buf_offset<i4_t>(&payload_, 4);
    ecefVY = buf_offset<i4_t>(&payload_, 8);
    ecefVZ = buf_offset<i4_t>(&payload_, 12);
    sAcc = buf_offset<u4_t>(&payload_, 16);
  }
  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }
  std::string to_string()
  {
    std::ostringstream oss;
    oss << "iTOW: " << iTOW;
    oss << " ecefVX: " << ecefVX;
    oss << " ecefVY: " << ecefVY;
    oss << " ecefVZ: " << ecefVZ;
    oss << " sAcc: " << sAcc;
    return oss.str();
  }
};
}  // namespace ubx::nav::velecef
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_VELECEF_HPP_
