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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_SAT_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_SAT_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::sat
{

struct flags_t
{
  union {
    u4_t all;
    struct
    {
      u1_t quality_ind : 3;
      l_t sv_used : 1;
      u1_t health : 2;
      l_t diff_corr : 1;
      l_t smoothed : 1;
      u1_t orbit_source : 3;
      l_t eph_avail : 1;
      l_t alm_avail : 1;
      l_t ano_avail : 1;
      l_t aop_avail : 1;
      l_t sbas_corr_used : 1;
      l_t rtcm_corr_used : 1;
      l_t slas_corr_used : 1;
      l_t spartn_corr_used : 1;
      l_t pr_corr_used : 1;
      l_t cr_corr_used : 1;
      l_t do_corr_used : 1;
      l_t clas_corr_used : 1;
    } bits;
  };
};

struct sat_data_t
{
  u1_t gnss_id;
  u1_t sv_id;
  u1_t cno;
  i1_t elev;
  i2_t azim;
  i2_t pr_res;
  flags_t flags;
};

class NavSatPayload : public UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_SAT;

  u4_t itow;
  u1_t version;
  u1_t num_svs;
  std::vector<sat_data_t> sat_data;

public:
  NavSatPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  NavSatPayload(u1_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    itow = buf_offset<u4_t>(&payload_, 0);
    version = buf_offset<u1_t>(&payload_, 4);
    num_svs = buf_offset<u1_t>(&payload_, 5);

    sat_data.clear();
    size_t offset = 8;  // The first satellite data starts at byte offset 8
    for (u1_t i = 0; i < num_svs; ++i) {
      sat_data_t sat;
      sat.gnss_id = buf_offset<u1_t>(&payload_, offset);
      sat.sv_id = buf_offset<u1_t>(&payload_, offset + 1);
      sat.cno = buf_offset<u1_t>(&payload_, offset + 2);
      sat.elev = buf_offset<i1_t>(&payload_, offset + 3);
      sat.azim = buf_offset<i2_t>(&payload_, offset + 4);
      sat.pr_res = buf_offset<i2_t>(&payload_, offset + 6);
      sat.flags.all = buf_offset<u4_t>(&payload_, offset + 8);

      sat_data.push_back(sat);
      offset += 12;  // Each satellite data takes up 12 bytes
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
    oss << ", num_svs: " << static_cast<int>(num_svs);

    for (size_t i = 0; i < sat_data.size(); ++i) {
      oss << "\n  sat " << i << ": ";
      oss << "gnss_id: " << static_cast<int>(sat_data[i].gnss_id);
      oss << ", sv_id: " << static_cast<int>(sat_data[i].sv_id);
      oss << ", cno: " << static_cast<int>(sat_data[i].cno);
      oss << ", elev: " << static_cast<int>(sat_data[i].elev);
      oss << ", azim: " << sat_data[i].azim;
      oss << ", pr_res: " << +sat_data[i].pr_res * 0.1;

      // Expanding the flags
      oss << ", flags: {";
      oss << "quality_ind: " << static_cast<int>(sat_data[i].flags.bits.quality_ind);
      oss << ", sv_used: " << static_cast<int>(sat_data[i].flags.bits.sv_used);
      oss << ", health: " << static_cast<int>(sat_data[i].flags.bits.health);
      oss << ", diff_corr: " << static_cast<int>(sat_data[i].flags.bits.diff_corr);
      oss << ", smoothed: " << static_cast<int>(sat_data[i].flags.bits.smoothed);
      oss << ", orbit_source: " << static_cast<int>(sat_data[i].flags.bits.orbit_source);
      oss << ", eph_avail: " << static_cast<int>(sat_data[i].flags.bits.eph_avail);
      oss << ", alm_avail: " << static_cast<int>(sat_data[i].flags.bits.alm_avail);
      oss << ", ano_avail: " << static_cast<int>(sat_data[i].flags.bits.ano_avail);
      oss << ", aop_avail: " << static_cast<int>(sat_data[i].flags.bits.aop_avail);
      oss << ", sbas_corr_used: " << static_cast<int>(sat_data[i].flags.bits.sbas_corr_used);
      oss << ", rtcm_corr_used: " << static_cast<int>(sat_data[i].flags.bits.rtcm_corr_used);
      oss << ", slas_corr_used: " << static_cast<int>(sat_data[i].flags.bits.slas_corr_used);
      oss << ", spartn_corr_used: " << static_cast<int>(sat_data[i].flags.bits.spartn_corr_used);
      oss << ", pr_corr_used: " << static_cast<int>(sat_data[i].flags.bits.pr_corr_used);
      oss << ", cr_corr_used: " << static_cast<int>(sat_data[i].flags.bits.cr_corr_used);
      oss << ", do_corr_used: " << static_cast<int>(sat_data[i].flags.bits.do_corr_used);
      oss << ", clas_corr_used: " << static_cast<int>(sat_data[i].flags.bits.clas_corr_used);
      oss << "}";
    }

    return oss.str();
  }
};

}  // namespace ubx::nav::sat

#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_SAT_HPP_
