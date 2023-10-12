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

#ifndef UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_RAWX_HPP_
#define UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_RAWX_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::rxm::rawx
{

struct trk_stat_t
{
  union {
    x1_t all;
    struct
    {
      l_t pr_valid : 1;
      l_t cp_valid : 1;
      l_t half_cyc : 1;
      l_t sub_half_cyc : 1;
    } bits;
  };
};

struct rec_stat_t
{
  union {
    x1_t all;
    struct
    {
      l_t leap_sec : 1;
      l_t clk_reset : 1;
    } bits;
  };
};

struct meas_data_t
{
  r8_t pr_mes;
  r8_t cp_mes;
  r4_t do_mes;
  u1_t gnss_id;
  u1_t sv_id;
  u1_t sig_id;
  u1_t freq_id;
  u2_t locktime;
  u1_t cno;
  u1_t pr_stdev;
  u1_t cp_stdev;
  u1_t do_stdev;
  trk_stat_t trk_stat;
};

class RxmRawxPayload : public UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_RXM;
  static const msg_id_t MSG_ID = UBX_RXM_RAWX;

  r8_t rcv_tow;
  u2_t week;
  i1_t leap_s;
  u1_t num_meas;
  rec_stat_t rec_stat;
  u1_t version;
  std::vector<meas_data_t> meas_data;

public:
  RxmRawxPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }


  RxmRawxPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    rcv_tow = buf_offset<r8_t>(&payload_, 0);
    week = buf_offset<u2_t>(&payload_, 8);
    leap_s = buf_offset<i1_t>(&payload_, 10);
    num_meas = buf_offset<u1_t>(&payload_, 11);
    rec_stat.all = buf_offset<x1_t>(&payload_, 12);

    version = buf_offset<u1_t>(&payload_, 13);

    meas_data.clear();
    size_t offset = 16;
    for (u1_t i = 0; i < num_meas; ++i) {
      meas_data_t meas;
      meas.pr_mes = buf_offset<r8_t>(&payload_, offset);
      meas.cp_mes = buf_offset<r8_t>(&payload_, offset + 8);
      meas.do_mes = buf_offset<r4_t>(&payload_, offset + 16);
      meas.gnss_id = buf_offset<u1_t>(&payload_, offset + 20);
      meas.sv_id = buf_offset<u1_t>(&payload_, offset + 21);
      meas.sig_id = buf_offset<u1_t>(&payload_, offset + 22);
      meas.freq_id = buf_offset<u1_t>(&payload_, offset + 23);
      meas.locktime = buf_offset<u2_t>(&payload_, offset + 24);
      meas.cno = buf_offset<u1_t>(&payload_, offset + 26);
      meas.pr_stdev = buf_offset<x1_t>(&payload_, offset + 27);
      meas.cp_stdev = buf_offset<x1_t>(&payload_, offset + 28);
      meas.do_stdev = buf_offset<x1_t>(&payload_, offset + 29);

      meas.trk_stat.all = buf_offset<x1_t>(&payload_, offset + 30);

      meas_data.push_back(meas);
      offset += 32;
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
    oss << "rcv_tow: " << rcv_tow << ", week: " << week << ", leap_s: " << static_cast<int>(leap_s);
    oss << ", num_meas: " << static_cast<int>(num_meas) << ", rec_stat: {leap_sec: " <<
      +rec_stat.bits.leap_sec;
    oss << ", clk_reset: " << +rec_stat.bits.clk_reset << "}, version: " <<
      static_cast<int>(version);

    for (size_t i = 0; i < meas_data.size(); ++i) {
      oss << "\n  meas " << i << ": ";
      oss << "pr_mes: " << meas_data[i].pr_mes << ", cp_mes: " << meas_data[i].cp_mes;
      oss << ", do_mes: " << meas_data[i].do_mes << ", gnss_id: " <<
        static_cast<int>(meas_data[i].gnss_id);
      oss << ", sv_id: " << static_cast<int>(meas_data[i].sv_id) << ", sig_id: " <<
        static_cast<int>(meas_data[i].sig_id);
      oss << ", freq_id: " << static_cast<int>(meas_data[i].freq_id) << ", locktime: " <<
        meas_data[i].locktime;
      oss << ", cno: " << static_cast<int>(meas_data[i].cno);
      oss << ", pr_stdev: " << +0.01 * pow(2, meas_data[i].pr_stdev);
      oss << ", cp_stdev: " << +0.004 * meas_data[i].cp_stdev;
      oss << ", do_stdev: " << +0.002 * pow(2, meas_data[i].do_stdev);
      oss << ", trk_stat: {pr_valid: " << static_cast<int>(meas_data[i].trk_stat.bits.pr_valid);
      oss << ", cp_valid: " << static_cast<int>(meas_data[i].trk_stat.bits.cp_valid);
      oss << ", half_cyc: " << static_cast<int>(meas_data[i].trk_stat.bits.half_cyc);
      oss << ", sub_half_cyc: " << static_cast<int>(meas_data[i].trk_stat.bits.sub_half_cyc) << "}";
    }

    return oss.str();
  }
};
}  // namespace ubx::rxm::rawx

#endif  // UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_RAWX_HPP_
