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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_SIG_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_SIG_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::sig
{
struct sig_flags_t
{
  union {
    x2_t all;
    struct
    {
      u1_t health : 2;
      l_t pr_smoothed : 1;
      l_t pr_used : 1;
      l_t cr_used : 1;
      l_t do_used : 1;
      l_t pr_corr_used : 1;
      l_t cr_corr_used : 1;
      l_t do_corr_used : 1;
      l_t auth_status : 1;
    } bits;
  };
};

struct sig_data_t
{
  u1_t gnss_id;
  u1_t sv_id;
  u1_t sig_id;
  u1_t freq_id;
  i2_t pr_res;
  u1_t cno;
  u1_t quality_ind;
  u1_t corr_source;
  u1_t iono_model;
  sig_flags_t sig_flags;
};

class NavSigPayload : public UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_SIG;

  u4_t itow;
  u1_t version;
  u1_t num_sigs;
  std::vector<sig_data_t> sig_data;

public:
  NavSigPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  NavSigPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    itow = buf_offset<u4_t>(&payload_, 0);
    version = buf_offset<u1_t>(&payload_, 4);
    num_sigs = buf_offset<u1_t>(&payload_, 5);

    sig_data.clear();
    size_t offset = 8;
    for (u1_t i = 0; i < num_sigs; ++i) {
      sig_data_t signal;
      signal.gnss_id = buf_offset<u1_t>(&payload_, offset);
      signal.sv_id = buf_offset<u1_t>(&payload_, offset + 1);
      signal.sig_id = buf_offset<u1_t>(&payload_, offset + 2);
      signal.freq_id = buf_offset<u1_t>(&payload_, offset + 3);
      signal.pr_res = buf_offset<i2_t>(&payload_, offset + 4);
      signal.cno = buf_offset<u1_t>(&payload_, offset + 6);
      signal.quality_ind = buf_offset<u1_t>(&payload_, offset + 7);
      signal.corr_source = buf_offset<u1_t>(&payload_, offset + 8);
      signal.iono_model = buf_offset<u1_t>(&payload_, offset + 9);
      signal.sig_flags.all = buf_offset<x2_t>(&payload_, offset + 10);

      sig_data.push_back(signal);
      offset += 16;  // Size of each repeated group is 16 bytes
    }
  }
  std::string to_string()
  {
    std::ostringstream oss;
    oss << "itow: " << itow << ", version: " << static_cast<int>(version);
    oss << ", num_sigs: " << static_cast<int>(num_sigs);

    for (size_t i = 0; i < sig_data.size(); ++i) {
      oss << "\n  sig " << i << ": ";
      oss << "gnss_id: " << static_cast<int>(sig_data[i].gnss_id);
      oss << ", sv_id: " << static_cast<int>(sig_data[i].sv_id);
      oss << ", sig_id: " << static_cast<int>(sig_data[i].sig_id);
      oss << ", freq_id: " << static_cast<int>(sig_data[i].freq_id);
      oss << ", pr_res: " << +sig_data[i].pr_res * 0.1;
      oss << ", cno: " << static_cast<int>(sig_data[i].cno);
      oss << ", quality_ind: " << static_cast<int>(sig_data[i].quality_ind);
      oss << ", corr_source: " << static_cast<int>(sig_data[i].corr_source);
      oss << ", iono_model: " << static_cast<int>(sig_data[i].iono_model);
      oss << ", sig_flags: {health: " << static_cast<int>(sig_data[i].sig_flags.bits.health);
      oss << ", pr_smoothed: " << static_cast<int>(sig_data[i].sig_flags.bits.pr_smoothed);
      oss << ", pr_used: " << static_cast<int>(sig_data[i].sig_flags.bits.pr_used);
      oss << ", cr_used: " << static_cast<int>(sig_data[i].sig_flags.bits.cr_used);
      oss << ", do_used: " << static_cast<int>(sig_data[i].sig_flags.bits.do_used);
      oss << ", pr_corr_used: " << static_cast<int>(sig_data[i].sig_flags.bits.pr_corr_used);
      oss << ", cr_corr_used: " << static_cast<int>(sig_data[i].sig_flags.bits.cr_corr_used);
      oss << ", do_corr_used: " << static_cast<int>(sig_data[i].sig_flags.bits.do_corr_used) << "}";
    }

    return oss.str();
  }
};
}  // namespace ubx::nav::sig

#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_SIG_HPP_
