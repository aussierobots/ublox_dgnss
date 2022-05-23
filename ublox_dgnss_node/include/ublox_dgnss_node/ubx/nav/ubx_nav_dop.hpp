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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_DOP_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_DOP_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::dop
{
class NavDOPPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_DOP;

  u4_t iTOW;     // ms - GPS Time of week of the navigation epoch.
  u2_t gDOP;     // 0.01 - Geometric DOP
  u2_t pDOP;     // 0.01 - Position DOP
  u2_t tDOP;     // 0.01 - Time DOP
  u2_t vDOP;     // 0.01 - Vertical DOP
  u2_t hDOP;     // 0.01 - Horizontal DOP
  u2_t nDOP;     // 0.01 - Northing DOP
  u2_t eDOP;     // 0.01 - Easting DOP

public:
  NavDOPPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  NavDOPPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    iTOW = buf_offset<u4_t>(&payload_, 0);
    gDOP = buf_offset<i4_t>(&payload_, 4);
    pDOP = buf_offset<i4_t>(&payload_, 6);
    tDOP = buf_offset<i4_t>(&payload_, 8);
    vDOP = buf_offset<i4_t>(&payload_, 10);
    hDOP = buf_offset<i4_t>(&payload_, 12);
    nDOP = buf_offset<i4_t>(&payload_, 14);
    eDOP = buf_offset<i4_t>(&payload_, 16);
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
    oss << std::setprecision(2);
    oss << " gDOP: " << +gDOP * 0.01;
    oss << " pDOP: " << +pDOP * 0.01;
    oss << " tDOP: " << +tDOP * 0.01;
    oss << " vDOP: " << +vDOP * 0.01;
    oss << " hDOP: " << +hDOP * 0.01;
    oss << " nDOP: " << +nDOP * 0.01;
    oss << " eDOP: " << +eDOP * 0.01;
    return oss.str();
  }
};
}  // namespace ubx::nav::dop
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_DOP_HPP_
