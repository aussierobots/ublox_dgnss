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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_ATT_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_ATT_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::att
{

class NavAttPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_ATT;

  u4_t iTOW;        // ms - GPS Time of week of the navigation epoch
  u1_t version;     // Message version (0 for this version)
  // reserved1[3] - Reserved
  i4_t roll;        // deg scale 1e-5 - Vehicle roll
  i4_t pitch;       // deg scale 1e-5 - Vehicle pitch
  i4_t heading;     // deg scale 1e-5 - Vehicle heading
  u4_t accRoll;     // deg scale 1e-5 - Vehicle roll accuracy
  u4_t accPitch;    // deg scale 1e-5 - Vehicle pitch accuracy
  u4_t accHeading;  // deg scale 1e-5 - Vehicle heading accuracy

public:
  NavAttPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  NavAttPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    iTOW = buf_offset<u4_t>(&payload_, 0);
    version = buf_offset<u1_t>(&payload_, 4);
    // reserved1[3] at offset 5-7
    roll = buf_offset<i4_t>(&payload_, 8);
    pitch = buf_offset<i4_t>(&payload_, 12);
    heading = buf_offset<i4_t>(&payload_, 16);
    accRoll = buf_offset<u4_t>(&payload_, 20);
    accPitch = buf_offset<u4_t>(&payload_, 24);
    accHeading = buf_offset<u4_t>(&payload_, 28);
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
    oss << " version: " << +version;
    oss << std::setprecision(5);
    oss << " roll: " << roll * 1e-5;
    oss << " pitch: " << pitch * 1e-5;
    oss << " heading: " << heading * 1e-5;
    oss << " accRoll: " << accRoll * 1e-5;
    oss << " accPitch: " << accPitch * 1e-5;
    oss << " accHeading: " << accHeading * 1e-5;

    return oss.str();
  }
};
}  // namespace ubx::nav::att
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_ATT_HPP_
