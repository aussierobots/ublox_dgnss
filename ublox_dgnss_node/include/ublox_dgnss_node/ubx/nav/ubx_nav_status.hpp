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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_STATUS_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_STATUS_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <bitset>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::status
{

enum gps_fix_t : u1_t {gps_no_fix = 0, gps_dead_reckoning_only = 1, gps_fix_2d = 2, gps_fix_3d = 3,
  gps_plus_dead_reackoning = 4, gps_time_only = 5};

enum map_matching_status_t : u1_t {none = 0,
  valid_not_used = 0b01,  // valid but not used, ie map matching data was received, but was too old
  valid_and_used = 0b10,  // valid and used, map matching data was applied
  valid_dead_reckoning = 0b11  // valid and used, map matching data was applied.
    // In case of sensor unavailability mapmatching data enables dead reckoning.
    // This requires map matched latitude/longitude orheading data.
};


enum psm_state_status_t : u1_t {acquisition = 0, tracking = 1,
  power_optimized_tracking = 2, inactive = 3};

enum spoof_det_state_t : u1_t {unknown_or_deactivated = 0, no_spoofing = 1, spoofing = 2,
  multiple_spoofing = 3};

enum carrier_solution_status_t : u1_t {no_carrier_range_solution = 0,
  carrier_phase_solution_with_floating_ambiguities = 1,
  carrier_phase_solution_with_fixed_ambiguities = 2 };

struct navigation_status_flags_t
{
  union {
    x1_t all;
    struct
    {
      l_t gpsFixOK : 1;     // 1 = position and velocity valid and within DOP and ACCMasks.
      l_t diffSoln : 1;     // 1 = differential corrections were applied
      l_t wknSet : 1;       // 1 = Week Number valid
      l_t towSet : 1;       // 1 = Time of Week valid
    } bits;
  };
};

struct navigation_fix_flags_t
{
  union {
    x1_t all;
    struct
    {
      l_t diffCorr : 1;                       // 1 = differential corrections available
      l_t carrSolnValid : 1;                  // 1 = valid carrSoln
      map_matching_status_t mapMatching : 2;  // map matching status
    } bits;
  };
};

struct navigation_status_flags2_t
{
  union {
    x1_t all;
    struct
    {
      psm_state_status_t psmState : 2;          // power save mode state
      spoof_det_state_t spoofDetState : 2;      // spoofing detection state
      carrier_solution_status_t carrSoln : 2;   // carrier phase range solution status
    } bits;
  };
};

class NavStatusPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_STATUS;

  u4_t iTOW;      // ms - GPS Time of week of the navigation epoch.
  gps_fix_t gpsFix;     // GPS Fix Type
  navigation_status_flags_t flags;     // navigation status flags
  navigation_fix_flags_t fixStat;     // fix status information
  navigation_status_flags2_t flags2;     // further information about navigation output
  u4_t ttff;     // ms - time to first fix (millisecond time tag)
  u4_t msss;     // ms  - milliseconds since statup/reset

public:
  NavStatusPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  NavStatusPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    iTOW = buf_offset<u4_t>(&payload_, 0);
    gpsFix = buf_offset<gps_fix_t>(&payload_, 4);
    flags = buf_offset<navigation_status_flags_t>(&payload_, 5);
    fixStat = buf_offset<navigation_fix_flags_t>(&payload_, 6);
    flags2 = buf_offset<navigation_status_flags2_t>(&payload_, 7);
    ttff = buf_offset<u4_t>(&payload_, 8);
    msss = buf_offset<u4_t>(&payload_, 12);
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
    oss << " gpsFix: " << +gpsFix;
    oss << " gpsFixOk: " << +flags.bits.gpsFixOK;
    oss << " diffSoln: " << +flags.bits.diffSoln;
    oss << " wknSet: " << +flags.bits.wknSet;
    oss << " towSet: " << +flags.bits.towSet;
    oss << " diffCorr: " << +fixStat.bits.diffCorr;
    oss << " carrSolnValid: " << +fixStat.bits.carrSolnValid;
    oss << " mapMatching: " << std::bitset<2>(fixStat.bits.mapMatching);
    oss << " psmState: " << +flags2.bits.psmState;
    oss << " spoofDetState: " << +flags2.bits.spoofDetState;
    oss << " carrSoln: " << +flags2.bits.carrSoln;
    oss << " ttff: " << +ttff;
    oss << " msss: " << +msss;

    return oss.str();
  }
};
}  // namespace ubx::nav::status
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_STATUS_HPP_
