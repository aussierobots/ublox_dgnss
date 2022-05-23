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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_PVT_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_PVT_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::pvt
{

struct validity_flags_t
{
  union {
    x1_t all;
    struct
    {
      l_t validDate : 1;                  // 1 = valid UTC date
      l_t validTime : 1;                  // 1 = valid UTC time of day
      l_t fullyResolved : 1;              // 1 = UTC Time of day has been fully resolved
                                          // (no seconds uncertainty)
      l_t validMag : 1;                   // 1 = valid magnetic declination
    } bits;
  };
};

enum gnss_fix_t : u1_t {gnss_no_fix = 0, gnss_dead_reackoning_only = 1, gnss_fix_2d = 2,
  gnss_fix_3d = 3,
  gnss_plus_dead_reackoning = 4, gnss_time_only = 5};


enum psm_state_pvt_t : u1_t {not_active = 0, enabled = 1, acquisition = 2, tracking = 3,
  power_optimized_tracking = 4, inactive = 5};


enum pvt_carrier_solution_status_t : u1_t {no_carrier_range_solution = 0,
  carrier_phase_solution_with_floating_ambiguities = 1,
  carrier_phase_solution_with_fixed_ambiguities = 2 };

struct gnss_fix_status_flags_t
{
  union {
    x1_t all;
    struct
    {
      l_t gnssFixOK : 1;                  // 1 = valid fix (ie within DOP & accruacy masks)
      l_t diffSoln : 1;                   // 1 = differential corrections were applied
      psm_state_pvt_t psmState : 3;       // power save mode state
      l_t headVehValid : 1;               // 1 = heading of vehicle is valid,
                                          // only set if the receiver is in sensor fusion mode
      pvt_carrier_solution_status_t carrSoln : 2;  // carrier phase range solution status
    } bits;
  };
};

struct additional_flags_1_t
{
  union {
    x1_t all;
    struct
    {
      l_t confirmedAvailable : 1;          // 1 = information about UTC date and time of day
                                           //     validity confirmation is available
      l_t confirmedDate : 1;               // 1 = UTC Date validity could be confirmed
      l_t confirmedTime : 1;               // 1 = UTC Time of Day validity could be confirmed
    } bits;
  };
};

struct additional_flags_2_t
{
  union {
    x1_t all;
    struct
    {
      l_t invalidLLH : 1;                   // 1 = invalid lon, lat, height, hMSL
    } bits;
  };
};

class NavPvtPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_PVT;

  u4_t iTOW;      // ms - GPS Time of week of the navigation epoch.
  u2_t year;     // y - year utc
  u1_t month;     // month - month utc
  u1_t day;     // d -day utc
  u1_t hour;     // h - hour utc
  u1_t min;     // mon - min utc
  u1_t sec;     // s - sec utc
  validity_flags_t valid;     // validity flags
  u4_t tAcc;     // ns - time accuracy estimate (utc)
  i4_t nano;     // ns - fraction of second, range -1e9 to 1e9 (utc)
  gnss_fix_t fixType;     // GNSSFix type
  gnss_fix_status_flags_t flags;     // fix status flags
  additional_flags_1_t flags2;     // additional flags
  u1_t numSV;     // Number of satellites used in Nav Solution
  i4_t lon;     // deg scale ie-7 - longitude
  i4_t lat;     // deg scale ie-7 - latitude
  i4_t height;     // mm - Height above ellipsoid
  i4_t hMSL;     // mm - Height above mean sea level
  u4_t hAcc;     // mm - Horizontal accuracy estimate
  u4_t vAcc;     // mm - Vertical accuracy estimate
  i4_t velN;     // mm/s - NED north velocity
  i4_t velE;     // mm/s - NED east velocity
  i4_t velD;     // mm/s - NED down velocity
  i4_t gSpeed;     // mm/s - ground speed (2-D)
  i4_t headMot;     // deg scale ie-5 - heading of motion (2-D)
  u4_t sAcc;     // mm/s - speed accuracy estimate
  u4_t headAcc;     // deg scale ie-5 - heading accruracy estime (both motion and vehicle)
  u2_t pDOP;     // scale 0.01 - Position DOP
  additional_flags_2_t flags3;     // additional flags
  u1_t reserved0[5];     // reserved
  i4_t headVeh;     // deg scale 1e-5 - Heading of vehicle (2-D),
                    // this is only valid when headVehValid is set,
                    // otherwise the output is set to the heading of motion
  i2_t magDec;     // deg scale 1e-2 - Magnetic declination. Only supported in ADR 4.10 and later
  u2_t magAcc;     // deg scale ie-2 - Mangetic declination accuracy.
                   // Only supported in ADR 4.10 and later

public:
  NavPvtPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  NavPvtPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    iTOW = buf_offset<u4_t>(&payload_, 0);
    year = buf_offset<u2_t>(&payload_, 4);
    month = buf_offset<u1_t>(&payload_, 6);
    day = buf_offset<u1_t>(&payload_, 7);
    hour = buf_offset<u1_t>(&payload_, 8);
    min = buf_offset<u1_t>(&payload_, 9);
    sec = buf_offset<u1_t>(&payload_, 10);
    valid.all = buf_offset<x1_t>(&payload_, 11);
    tAcc = buf_offset<u4_t>(&payload_, 12);
    nano = buf_offset<i4_t>(&payload_, 16);
    fixType = buf_offset<gnss_fix_t>(&payload_, 20);
    flags = buf_offset<gnss_fix_status_flags_t>(&payload_, 21);
    flags2 = buf_offset<additional_flags_1_t>(&payload_, 22);
    numSV = buf_offset<u1_t>(&payload_, 23);
    lon = buf_offset<i4_t>(&payload_, 24);
    lat = buf_offset<i4_t>(&payload_, 28);
    height = buf_offset<i4_t>(&payload_, 32);
    hMSL = buf_offset<i4_t>(&payload_, 36);
    hAcc = buf_offset<u4_t>(&payload_, 40);
    vAcc = buf_offset<u4_t>(&payload_, 44);
    velN = buf_offset<i4_t>(&payload_, 48);
    velE = buf_offset<i4_t>(&payload_, 52);
    velD = buf_offset<i4_t>(&payload_, 56);
    gSpeed = buf_offset<i4_t>(&payload_, 60);
    headMot = buf_offset<i4_t>(&payload_, 64);
    sAcc = buf_offset<u4_t>(&payload_, 68);
    headAcc = buf_offset<u4_t>(&payload_, 72);
    pDOP = buf_offset<u2_t>(&payload_, 76);
    flags3 = buf_offset<additional_flags_2_t>(&payload_, 78);
    headVeh = buf_offset<i4_t>(&payload_, 84);
    magDec = buf_offset<i2_t>(&payload_, 88);
    magAcc = buf_offset<u2_t>(&payload_, 90);
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
    oss << " date time: " << std::setw(2) << +year << "/" << std::setw(2) << +month << "/" <<
      std::setw(2) << +day;
    oss << " " << std::setw(2) << +hour << ":" << std::setw(2) << +min << ":" << std::setw(2) <<
      +sec;
    oss << " validDate: " << +valid.bits.validDate;
    oss << " validTime: " << +valid.bits.validTime;
    oss << " fullyResolved: " << +valid.bits.fullyResolved;
    oss << " validMag: " << +valid.bits.validMag;
    oss << " tAcc: " << +tAcc;
    oss << " nano: " << +nano;
    oss << " fixType: " << +fixType;
    oss << " gnssFixOK:" << +flags.bits.gnssFixOK;
    oss << " diffSoln:" << +flags.bits.diffSoln;
    oss << " psmState:" << +flags.bits.psmState;
    oss << " headVehValid:" << +flags.bits.headVehValid;
    oss << " carrSoln:" << +flags.bits.carrSoln;
    oss << " confirmedAvailable: " << +flags2.bits.confirmedAvailable;
    oss << " confirmedDate: " << +flags2.bits.confirmedDate;
    oss << " confirmedTime: " << +flags2.bits.confirmedTime;
    oss << " numSV: " << +numSV;
    oss << std::fixed;
    oss << std::setprecision(7);
    oss << " lon: " << +lon << " " << lon * 1e-7;
    oss << " lat: " << +lat << " " << lat * 1e-7;
    oss << std::setprecision(0);
    oss << " height: " << +height;
    oss << " hMSL: " << +hMSL;
    oss << " hAcc: " << +hAcc;
    oss << " vAcc: " << +vAcc;
    oss << " velN: " << +velN;
    oss << " velE: " << +velE;
    oss << " velD: " << +velD;
    oss << " gSpeed: " << +gSpeed;
    oss << std::setprecision(5);
    oss << " headMot: " << +headMot * 1e-5;
    oss << std::setprecision(0);
    oss << " sAcc: " << +sAcc;
    oss << std::setprecision(5);
    oss << " headAcc: " << +headAcc * 1e-5;
    oss << std::setprecision(2);
    oss << " pDOP: " << +pDOP * 0.01;
    oss << std::setprecision(0);
    oss << " invalidLlh: " << +flags3.bits.invalidLLH;
    oss << std::setprecision(5);
    oss << " headVeh: " << +headVeh * 1e-5;
    oss << std::setprecision(2);
    oss << " magDec: " << +magDec * 1e-2;
    oss << " magAcc: " << +magAcc * 1e-2;

    return oss.str();
  }
};
}    // namespace ubx::nav::pvt
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_PVT_HPP_
