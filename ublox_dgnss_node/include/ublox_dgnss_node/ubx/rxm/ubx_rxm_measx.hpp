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

#ifndef UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_MEASX_HPP_
#define UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_MEASX_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::rxm::measx
{

enum tow_set_t : u1_t {tow_not_set = 0, tow_set = 1, tow_set2 = 2};

enum mpath_indic_t : u1_t
{
  mpath_not_measured = 0,
  mpath_low = 1,
  mpath_medium = 2,
  mpath_high = 3
};

struct sv_data_t
{
  u1_t gnss_id;
  u1_t sv_id;
  u1_t c_no;
  mpath_indic_t mpath_indic;
  i4_t doppler_ms;
  i4_t doppler_hz;
  u2_t whole_chips;
  u2_t frac_chips;
  u4_t code_phase;
  u1_t int_code_phase;
  u1_t pseu_range_rms_err;
  // u1_t reserved_4[2];
};

class RxmMeasxPayload : public UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_RXM;
  static const msg_id_t MSG_ID = UBX_RXM_MEASX;

  u1_t version;
  // u1_t reserved_0[3];
  u4_t gps_tow;
  u4_t glo_tow;
  u4_t bds_tow;
  // u1_t reserved_1[4];
  u4_t qzss_tow;
  u2_t gps_tow_acc;
  u2_t glo_tow_acc;
  u2_t bds_tow_acc;
  // u1_t reserved_2[2];
  u2_t qzss_tow_acc;
  u1_t num_sv;
  tow_set_t tow_set;
  // u1_t reserved_3[8];
  std::vector<sv_data_t> sv_data;

public:
  RxmMeasxPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  RxmMeasxPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    version = buf_offset<u1_t>(&payload_, 0);
    // memcpy(reserved_0, &payload_[1], 3);
    gps_tow = buf_offset<u4_t>(&payload_, 4);
    glo_tow = buf_offset<u4_t>(&payload_, 8);
    bds_tow = buf_offset<u4_t>(&payload_, 12);
    // memcpy(reserved_1, &payload_[16], 4);
    qzss_tow = buf_offset<u4_t>(&payload_, 20);
    gps_tow_acc = buf_offset<u2_t>(&payload_, 24);
    glo_tow_acc = buf_offset<u2_t>(&payload_, 26);
    bds_tow_acc = buf_offset<u2_t>(&payload_, 28);
    // memcpy(reserved_2, &payload_[30], 2);
    qzss_tow_acc = buf_offset<u2_t>(&payload_, 32);
    num_sv = buf_offset<u1_t>(&payload_, 34);
    tow_set = static_cast<tow_set_t>(buf_offset<u1_t>(&payload_, 35) & 0x03);
    // memcpy(reserved_3, &payload_[36], 8);

    // Filling in the repeated block of satellite data
    sv_data.clear();
    size_t offset = 44;
    for (u1_t i = 0; i < num_sv; ++i) {
      sv_data_t sv;
      sv.gnss_id = buf_offset<u1_t>(&payload_, offset);
      sv.sv_id = buf_offset<u1_t>(&payload_, offset + 1);
      sv.c_no = buf_offset<u1_t>(&payload_, offset + 2);
      sv.mpath_indic = static_cast<mpath_indic_t>(buf_offset<u1_t>(&payload_, offset + 3));
      sv.doppler_ms = buf_offset<i4_t>(&payload_, offset + 4);
      sv.doppler_hz = buf_offset<i4_t>(&payload_, offset + 8);
      sv.whole_chips = buf_offset<u2_t>(&payload_, offset + 12);
      sv.frac_chips = buf_offset<u2_t>(&payload_, offset + 14);
      sv.code_phase = buf_offset<u4_t>(&payload_, offset + 16);
      sv.int_code_phase = buf_offset<u1_t>(&payload_, offset + 20);
      sv.pseu_range_rms_err = buf_offset<u1_t>(&payload_, offset + 21);
      // memcpy(sv.reserved_4, &payload_[offset + 22], 2);
      sv_data.push_back(sv);
      offset += 24;
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
    oss << "version: " << +version;
    oss << " gps_tow: " << gps_tow;
    oss << " glo_tow: " << glo_tow;
    oss << " bds_tow: " << bds_tow;
    oss << " qzss_tow: " << qzss_tow;
    oss << " gps_tow_acc: " << +(gps_tow_acc * std::pow(2, -4));
    oss << " glo_tow_acc: " << +(glo_tow_acc * std::pow(2, -4));
    oss << " bds_tow_acc: " << +(bds_tow_acc * std::pow(2, -4));
    oss << " qzss_tow_acc: " << +(qzss_tow_acc * std::pow(2, -4));
    oss << " num_sv: " << +num_sv;
    oss << " tow_set: " << +static_cast<uint8_t>(tow_set);

    // Iterate through the satellite data array and append each satellite's data to the string
    for (size_t i = 0; i < sv_data.size(); ++i) {
      oss << "\n  Sat " << i + 1 << ": ";
      oss << "gnss_id: " << +sv_data[i].gnss_id;
      oss << " sv_id: " << +sv_data[i].sv_id;
      oss << " c_no: " << +sv_data[i].c_no;
      oss << " mpath_indic: " << static_cast<uint8_t>(sv_data[i].mpath_indic);
      oss << " doppler_ms: " << +(sv_data[i].doppler_ms * 0.04);
      oss << " doppler_hz: " << +(sv_data[i].doppler_hz * 0.2);
      oss << " whole_chips: " << sv_data[i].whole_chips;
      oss << " frac_chips: " << sv_data[i].frac_chips;
      oss << " code_phase: " << +(sv_data[i].code_phase * std::pow(2, -21));
      oss << " int_code_phase: " << +sv_data[i].int_code_phase;
      oss << " pseu_range_rms_err: " << +sv_data[i].pseu_range_rms_err;
    }

    return oss.str();
  }
};
}  // namespace ubx::rxm::measx

#endif  // UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_MEASX_HPP_
