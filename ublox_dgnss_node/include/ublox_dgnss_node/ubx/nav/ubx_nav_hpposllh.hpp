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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_HPPOSLLH_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_HPPOSLLH_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/utils.hpp"
#include "ublox_dgnss_node/ubx/ubx.hpp"

namespace ubx::nav::hpposllh
{

struct invalid_llh_t
{
  union {
    x1_t all;
    struct
    {
      l_t invalid_lon : 1;
      l_t invalid_lat : 1;
      l_t invalid_height : 1;
      l_t invalid_hMSL : 1;
      l_t invalid_lonHp : 1;
      l_t invalid_latHp : 1;
      l_t invalid_heightHp : 1;
      l_t invalid_hMSLHp : 1;
    } bits;
  };
};

class NavHPPosLLHPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_HPPOSLLH;

  u1_t version;     // message version (0x00 for this version)
  u1_t reserved0[2];     // reserved
  invalid_llh_t flags;     // 1 = Invalid lon, lat, height, hMSL, lonHp, latHp, heightHp and hMSLHp
  u4_t iTOW;      // ms - GPS Time of week of the navigation epoch.
  i4_t lon;     // deg scale 1e-7 - longitude
  i4_t lat;     // deg scale 1e-7 - latitude
  i4_t height;     // mm - Height above ellipsoid
  i4_t hMSL;     // mm - Height above mean sea level
  i1_t lonHp;     // deg scale 1e-9 - Precision longitude in deg * 1e-7 = lon +(lonHp * 1e-2)
  i1_t latHp;     // deg scale 1e-9 - Precise latitude in deg  *  1e-7  =  lat  +(latHp * 1e-2)
  i1_t heightHp;     // mm scale 0.1 - Precise height in mm = height + (heightHp * 0.1)
  i1_t hMSLHp;     // mm scale 0.1 - Precise height in mm =hMSL + (hMSLHp * 0.1)
  u4_t hAcc;      // mm scale 0.1 - horizontal accuracy estimate
  u4_t vAcc;      // mm scale 0.1 - vertical accuracy estimate

public:
  NavHPPosLLHPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  NavHPPosLLHPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    version = payload_[0];
    flags.all = buf_offset<x1_t>(&payload_, 3);
    iTOW = buf_offset<u4_t>(&payload_, 4);
    lon = buf_offset<i4_t>(&payload_, 8);
    lat = buf_offset<i4_t>(&payload_, 12);
    height = buf_offset<i4_t>(&payload_, 16);
    hMSL = buf_offset<i4_t>(&payload_, 20);
    lonHp = buf_offset<i1_t>(&payload_, 24);
    latHp = buf_offset<i1_t>(&payload_, 25);
    heightHp = buf_offset<i1_t>(&payload_, 26);
    hMSLHp = buf_offset<i1_t>(&payload_, 27);
    hAcc = buf_offset<u4_t>(&payload_, 28);
    vAcc = buf_offset<u4_t>(&payload_, 32);
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
    oss << " lon: " << +lon;
    oss << " lat: " << +lat;
    oss << " height: " << +height;
    oss << " hMSL: " << +hMSL;
    oss << "  - Hp";
    oss << std::setprecision(9);
    double lonHp_deg = lonHp * 1e-9;
    double latHp_deg = latHp * 1e-9;
    oss << " lonHp(deg): " << +lonHp_deg;
    oss << " latHp(deg): " << +latHp_deg;
    oss << std::setprecision(9);
    oss << " lon: " << +((lon * 1e-7) + (lonHp_deg));
    oss << " lat: " << +((lat * 1e-7) + (latHp_deg));
    oss << std::setprecision(1);
    oss << " height: " << +((height * 0.1) + (heightHp * 0.1));
    oss << " hMSL: " << +((hMSL * 0.1) + (hMSLHp * 0.1));
    oss << " hAcc: " << +hAcc * 0.1;
    oss << " vAcc: " << +vAcc * 0.1;
    oss << " - flags invalid lon: " << +flags.bits.invalid_lon;
    oss << " lat: " << +flags.bits.invalid_lat;
    oss << " height: " << +flags.bits.invalid_height;
    oss << " hMSL: " << +flags.bits.invalid_hMSL;
    oss << " lonHp: " << +flags.bits.invalid_lonHp;
    oss << " lonHp: " << +flags.bits.invalid_latHp;
    oss << " heightHp: " << +flags.bits.invalid_heightHp;
    oss << " hMSLHp: " << +flags.bits.invalid_hMSLHp;

    return oss.str();
  }
};
}  // namespace ubx::nav::hpposllh
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_HPPOSLLH_HPP_
