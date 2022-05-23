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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_POSLLH_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_POSLLH_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::posllh
{

class NavPosLLHPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_POSLLH;

  u4_t iTOW;      // ms - GPS Time of week of the navigation epoch.
  i4_t lon;       // deg scale 1e-7 - longitude
  i4_t lat;       // deg scale ie-7 = latitude
  i4_t height;     // mm - Height above ellipsoid
  i4_t hMSL;     // mm - Height above mean sea level
  u4_t hAcc;     // mm - Horizontal accuracy estimate
  u4_t vAcc;     // mm - Vertical accuracy estimate

public:
  NavPosLLHPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  NavPosLLHPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    iTOW = buf_offset<u4_t>(&payload_, 0);
    lon = buf_offset<i4_t>(&payload_, 4);
    lat = buf_offset<i4_t>(&payload_, 8);
    height = buf_offset<i4_t>(&payload_, 12);
    hMSL = buf_offset<i4_t>(&payload_, 16);
    hAcc = buf_offset<u4_t>(&payload_, 20);
    vAcc = buf_offset<u4_t>(&payload_, 24);
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
    oss << std::fixed;
    oss << std::setprecision(7);
    oss << " lon: " << lon << " " << lon * 1e-7;
    oss << " lat: " << lat << " " << lat * 1e-7;
    oss << std::setprecision(0);
    oss << " height: " << height;
    oss << " hMSL: " << hMSL;
    oss << " hAcc: " << hAcc;
    oss << " vAcc: " << vAcc;
    return oss.str();
  }
};
}    // namespace ubx::nav::posllh
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_POSLLH_HPP_
