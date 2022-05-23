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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_TIMEUTC_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_TIMEUTC_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::timeutc
{

enum utc_std_id_t : u1_t
{
  not_available = 0,
  CRL = 1,                // Communications Research Labratory (CRL),Tokyo, Japan
  NIST = 2,               // National Institute of Standards andTechnology (NIST)
  USNO = 3,               // U.S. Naval Observatory (USNO)
  BIPM = 4,               // International Bureau of Weights andMeasures (BIPM)
  EURO = 5,               // European laboratories
  SU = 6,                 // Former Soviet Union
  NTSC = 7,               // National Time Service Center (NTSC) China
  UTC_UNKNOWN = 15        // Unknown time source
};

struct validity_flags_t
{
  union {
    x1_t all;
    struct
    {
      l_t validTOW : 1;                   // 1 = valid Time of Week
      l_t validWKN : 1;                   // 1 = valid Week Number
      l_t validUTC : 1;                   // 1 = valid UTC Time
      utc_std_id_t utcStandard : 4;                   // UTC standard identifier
    } bits;
  };
};

class NavTimeUTCPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_TIMEUTC;

  u4_t iTOW;      // ms - GPS Time of week of the navigation epoch.
  u4_t tAcc;     // ns - time accuracy estimate (utc)
  i4_t nano;     // ns - fraction of second, range -1e9 to 1e9 (utc)
  u2_t year;     // y - year utc
  u1_t month;     // month - month utc
  u1_t day;     // d -day utc
  u1_t hour;     // h - hour utc
  u1_t min;     // min - min utc
  u1_t sec;     // s - sec utc
  validity_flags_t valid;     // validity flags

public:
  NavTimeUTCPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  NavTimeUTCPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    iTOW = buf_offset<u4_t>(&payload_, 0);
    tAcc = buf_offset<u4_t>(&payload_, 4);
    nano = buf_offset<i4_t>(&payload_, 8);
    year = buf_offset<u2_t>(&payload_, 12);
    month = buf_offset<u1_t>(&payload_, 14);
    day = buf_offset<u1_t>(&payload_, 15);
    hour = buf_offset<u1_t>(&payload_, 16);
    min = buf_offset<u1_t>(&payload_, 17);
    sec = buf_offset<u1_t>(&payload_, 18);
    valid = buf_offset<validity_flags_t>(&payload_, 19);
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
    oss << std::setfill('0');
    oss << " tAcc: " << +tAcc;
    oss << " nano: " << +nano;
    oss << " date time: " << std::setw(2) << +year << "/" << std::setw(2) << +month << "/" <<
      std::setw(2) << +day;
    oss << " " << std::setw(2) << +hour << ":" << std::setw(2) << +min << ":" << std::setw(2) <<
      +sec;
    oss << " validTOW: " << +valid.bits.validTOW;
    oss << " validWKN: " << +valid.bits.validWKN;
    oss << " validUTC: " << +valid.bits.validUTC;
    oss << " utcStanard: " << +valid.bits.utcStandard;

    return oss.str();
  }
};
}  // namespace ubx::nav::timeutc
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_TIMEUTC_HPP_
