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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_ODO_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_ODO_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::odo
{

class NavOdoPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_ODO;

  u1_t version;     // message version (0x00 for this version)
  u1_t reserved0[3];     // reserved
  u4_t iTOW;      // ms - GPS Time of week of the navigation epoch.
  u4_t distance;     // m - ground distance since last reset
  u4_t totalDistance;     // m - total comulative ground distance (since last cold start)
  u4_t distanceStd;     // m - ground distance accuracy (1-sigma)

public:
  NavOdoPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  NavOdoPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    version = payload_[0];
    iTOW = buf_offset<u4_t>(&payload_, 4);
    distance = buf_offset<i4_t>(&payload_, 8);
    totalDistance = buf_offset<i4_t>(&payload_, 12);
    distanceStd = buf_offset<u4_t>(&payload_, 16);
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
    oss << " distance: " << distance;
    oss << " totalDistance: " << totalDistance;
    oss << " distanceStd: " << distanceStd;
    return oss.str();
  }
};
}  // namespace ubx::nav::odo
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_ODO_HPP_
