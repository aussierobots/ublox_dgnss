// Copyright 2023 Australian Robotics Supplies & Technology
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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_ORB_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_ORB_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::orb
{

struct sv_flag_t
{
  union {
    x1_t all;
    struct
    {
      u1_t health : 2;
      u1_t visibility : 2;
    } bits;
  };
};

struct eph_t
{
  union {
    x1_t all;
    struct
    {
      u1_t eph_usability : 5;
      u1_t eph_source : 3;
    } bits;
  };
};

struct alm_t
{
  union {
    x1_t all;
    struct
    {
      u1_t alm_usability : 5;
      u1_t alm_source : 3;
    } bits;
  };
};

struct other_orb_t
{
  union {
    x1_t all;
    struct
    {
      u1_t ano_aop_usability : 5;
      u1_t orb_type : 3;
    } bits;
  };
};

struct sv_info_t
{
  u1_t gnss_id;
  u1_t sv_id;
  sv_flag_t sv_flag;
  eph_t eph;
  alm_t alm;
  other_orb_t other_orb;
};

class NavOrbPayload : public UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_ORB;

  u4_t itow;
  u1_t version;
  u1_t num_sv;
  std::vector<sv_info_t> sv_info;

public:
  NavOrbPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  NavOrbPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    itow = buf_offset<u4_t>(&payload_, 0);
    version = buf_offset<u1_t>(&payload_, 4);
    num_sv = buf_offset<u1_t>(&payload_, 5);

    sv_info.clear();
    size_t offset = 8;
    for (u1_t i = 0; i < num_sv; ++i) {
      sv_info_t info;
      info.gnss_id = buf_offset<u1_t>(&payload_, offset);
      info.sv_id = buf_offset<u1_t>(&payload_, offset + 1);
      info.sv_flag.all = buf_offset<x1_t>(&payload_, offset + 2);
      info.eph.all = buf_offset<x1_t>(&payload_, offset + 3);
      info.alm.all = buf_offset<x1_t>(&payload_, offset + 4);
      info.other_orb.all = buf_offset<x1_t>(&payload_, offset + 5);

      sv_info.push_back(info);
      offset += 6;
    }
  }
  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }
  std::string to_string()
  {
    std::ostringstream oss;
    oss << "itow: " << itow << ", version: " << static_cast<int>(version);
    oss << ", num_sv: " << static_cast<int>(num_sv);

    for (size_t i = 0; i < sv_info.size(); ++i) {
      oss << "\n  sv_info " << i << ": ";
      oss << "gnss_id: " << static_cast<int>(sv_info[i].gnss_id);
      oss << ", sv_id: " << static_cast<int>(sv_info[i].sv_id);
      oss << ", health: " << static_cast<int>(sv_info[i].sv_flag.bits.health);
      oss << ", visibility: " << static_cast<int>(sv_info[i].sv_flag.bits.visibility);
      oss << ", eph_usability: " << static_cast<int>(sv_info[i].eph.bits.eph_usability);
      oss << ", eph_source: " << static_cast<int>(sv_info[i].eph.bits.eph_source);
      oss << ", alm_usability: " << static_cast<int>(sv_info[i].alm.bits.alm_usability);
      oss << ", alm_source: " << static_cast<int>(sv_info[i].alm.bits.alm_source);
      oss << ", ano_aop_usability: " <<
        static_cast<int>(sv_info[i].other_orb.bits.ano_aop_usability);
      oss << ", orb_type: " << static_cast<int>(sv_info[i].other_orb.bits.orb_type);
    }

    return oss.str();
  }
};

}  // namespace ubx::nav::orb

#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_ORB_HPP_
