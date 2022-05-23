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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_RELPOSNED_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_RELPOSNED_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::relposned
{


enum carrier_solution_status_t : u1_t {no_carrier_range_solution = 0,
  carrier_phase_solution_with_floating_ambiguities = 1,
  carrier_phase_solution_with_fixed_ambiguities = 2 };

struct status_flags_t
{
  union {
    x4_t all;
    struct
    {
      l_t gnssFixOK : 1;                // 1 = A valid fix (i.e within DOP & accuracy masks)
      l_t diffSoln : 1;                 // 1 = differential corrections were applied
      l_t relPosValid : 1;              // 1 = relative position components and accuracies are valid
                                        // and, in moving base mode only, if baseline is valid
      carrier_solution_status_t carrSoln : 2;  // carrier phase range solution
      l_t isMoving : 1;                 // 1 if the received is operating in moving base mode
      l_t refPosMiss : 1;               // 1 if extrapolated reference position was used to compute
                                        // moving base solution this epoch.
      l_t refObsMiss : 1;               // 1 if extrapolated reference observations were used to
                                        // compute moving base solution this epoch.
      l_t relPosHeadingValid : 1;       // 1 = relPosHeading is valid
      l_t relPosNormalized : 1;         // 1 = components of the relative position vector
                                        // (including the high-precision parts) are normalized
    } bits;
  };
};

class NavRelPosNedPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_RELPOSNED;

  u1_t version;     // message version (0x01 for this version)
  u1_t reserved0;     // reserved
  u2_t refStationId;     // reference station ID. Must be in the range 0..4095
  u4_t iTOW;      // ms - GPS Time of week of the navigation epoch.
  i4_t relPosN;      // cm - North component of relative position vector
  i4_t relPosE;      // cm - East component of relative position vector
  i4_t relPosD;      // cm - Down component of relative position vector
  i4_t relPosLength;    // cm - Length of the relative position vector
  i4_t relPosHeading;    // deg scale 1e-5 - Heading of the relative position vector
  u1_t reserved1[4];     // reserved
  i1_t relPosHPN;      // mm scale 0.1 - full HP North is given by relPosN + (relPosHPN * 1e-2)
  i1_t relPosHPE;      // mm scale 0.1 - full HP East is given by relPosE + (relPosHPE * 1e-2)
  i1_t relPosHPD;      // mm scale 0.1 - full HP Down is given by relPosD + (relPosHPD * 1e-2)
  i1_t relPosHPLength;      // mm scale 0.1 - full HP length is given by
                            //  relPosLength + (relPosHPLength * 1e-2)
  u4_t accN;      // mm scale 0.1 - Accuracy of relative position North Component
  u4_t accE;      // mm scale 0.1 - Accuracy of relative position East Component
  u4_t accD;      // mm scale 0.1 - Accuracy of relative position Down Component
  u4_t accLength;      // mm scale 0.1 - Accuracy of length of the relative position vector
  u4_t accHeading;      // deg scale 1e-5 - Accuracy of heading of the relative position vector
  u1_t reserved2[4];     // reserved
  status_flags_t flags;     // navigation status flags

public:
  NavRelPosNedPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  NavRelPosNedPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    version = buf_offset<u1_t>(&payload_, 0);
    refStationId = buf_offset<u2_t>(&payload_, 2);
    iTOW = buf_offset<u4_t>(&payload_, 4);
    relPosN = buf_offset<i4_t>(&payload_, 8);
    relPosE = buf_offset<i4_t>(&payload_, 12);
    relPosD = buf_offset<i4_t>(&payload_, 16);
    relPosLength = buf_offset<i4_t>(&payload_, 20);
    relPosHeading = buf_offset<i4_t>(&payload_, 24);
    relPosHPN = buf_offset<i1_t>(&payload_, 32);
    relPosHPE = buf_offset<i1_t>(&payload_, 33);
    relPosHPD = buf_offset<i1_t>(&payload_, 34);
    relPosHPLength = buf_offset<i1_t>(&payload_, 35);
    accN = buf_offset<u4_t>(&payload_, 36);
    accE = buf_offset<u4_t>(&payload_, 40);
    accD = buf_offset<u4_t>(&payload_, 44);
    accLength = buf_offset<u4_t>(&payload_, 48);
    accHeading = buf_offset<u4_t>(&payload_, 52);
    flags = buf_offset<status_flags_t>(&payload_, 60);
  }
  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }
  std::string to_string()
  {
    std::ostringstream oss;
    oss << std::fixed;
    oss << "ver: " << +version;
    oss << " refStationId: " << +refStationId;
    oss << " iTOW: " << iTOW;
    oss << " relPos - N: " << +relPosN;
    oss << " E: " << +relPosE;
    oss << " D: " << +relPosD;
    oss << " length: " << +relPosLength;
    oss << std::setprecision(5);
    oss << " heading:" << +relPosHeading * 1e-5;
    oss << std::setprecision(1);
    oss << " relPosHP - N: " << +relPosHPN * 0.1;
    oss << " E: " << +relPosHPE * 0.1;
    oss << " D: " << +relPosHPD * 0.1;
    oss << " length: " << +relPosHPLength * 0.1;
    oss << std::setprecision(2);
    oss << " precise HP - N: " << +(relPosN + (relPosHPN * 1e-2));
    oss << " E: " << +(relPosE + (relPosHPE * 1e-2));
    oss << " D: " << +(relPosD + (relPosHPD * 1e-2));
    oss << " length: " << +(relPosLength + (relPosHPLength * 1e-2));
    oss << std::setprecision(1);
    oss << " acc - N: " << +accN * 0.1;
    oss << " E: " << +accE * 0.1;
    oss << " D: " << +accD * 0.1;
    oss << " length: " << +accLength * 0.1;
    oss << std::setprecision(5);
    oss << " heading: " << +accHeading * 1e-5;
    oss << " flags - ";
    oss << std::setprecision(0);
    oss << " gnssFixOK: " << +flags.bits.gnssFixOK;
    oss << " diffSoln: " << +flags.bits.diffSoln;
    oss << " relPosValid: " << +flags.bits.relPosValid;
    oss << " carrSoln: " << +flags.bits.carrSoln;
    oss << " isMoving: " << +flags.bits.isMoving;
    oss << " refPosMiss: " << +flags.bits.refPosMiss;
    oss << " refObsMiss: " << +flags.bits.refObsMiss;
    oss << " relPosHeadingValid: " << +flags.bits.relPosHeadingValid;
    oss << " relPosNormalized: " << +flags.bits.relPosNormalized;

    return oss.str();
  }
};
}  // namespace ubx::nav::relposned
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_RELPOSNED_HPP_
