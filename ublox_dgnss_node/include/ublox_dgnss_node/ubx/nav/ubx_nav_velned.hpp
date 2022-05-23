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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_VELNED_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_VELNED_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::velned
{

class NavVelNEDPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_VELNED;

  u4_t iTOW;      // ms - GPS Time of week of the navigation epoch.
  i4_t velN;     // cm/s - NED north velocity
  i4_t velE;     // cm/s - NED east velocity
  i4_t velD;     // cm/s - NED down velocity
  u4_t speed;     // cm/s - speed (3-D)
  u4_t gSpeed;     // cm/s - ground speed (2-D)
  i4_t heading;     // deg scale ie-5 - heading of motion (2-D)
  u4_t sAcc;     // cm/s - speed accuracy estimate
  u4_t cAcc;     // deg scale ie-5 - course/heading accruracy estime (both motion and vehicle)

public:
  NavVelNEDPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  NavVelNEDPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    iTOW = buf_offset<u4_t>(&payload_, 0);
    velN = buf_offset<i4_t>(&payload_, 4);
    velE = buf_offset<i4_t>(&payload_, 8);
    velD = buf_offset<i4_t>(&payload_, 12);
    speed = buf_offset<i4_t>(&payload_, 16);
    gSpeed = buf_offset<i4_t>(&payload_, 20);
    heading = buf_offset<i4_t>(&payload_, 24);
    sAcc = buf_offset<u4_t>(&payload_, 28);
    cAcc = buf_offset<u4_t>(&payload_, 32);
  }
  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }
  std::string to_string()
  {
    std::ostringstream oss;
    oss << std::fixed;
    oss << "iTOW: " << iTOW;
    oss << " velN: " << +velN;
    oss << " velE: " << +velE;
    oss << " velD: " << +velD;
    oss << " speed: " << +speed;
    oss << " gSpeed: " << +gSpeed;
    oss << std::setprecision(5);
    oss << " heading: " << +heading * 1e-5;
    oss << std::setprecision(0);
    oss << " sAcc: " << +sAcc;
    oss << std::setprecision(5);
    oss << " cAcc: " << +cAcc * 1e-5;

    return oss.str();
  }
};
}  // namespace ubx::nav::velned
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_VELNED_HPP_
