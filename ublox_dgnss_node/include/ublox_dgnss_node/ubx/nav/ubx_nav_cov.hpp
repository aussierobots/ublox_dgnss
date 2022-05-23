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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_COV_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_COV_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

// NOTE this message doesnt appear in the F9P Interface Description but is available in u-center
// This message outputs the covariance matrices for the position and velocity
// solutions in the topocentric coordinate system defined as the local-level North
// (N), East (E), Down (D) frame. As the covariance matrices are symmetric, only
// the upper triangular part is output.
namespace ubx::nav::cov
{
class NavCovPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_COV;

  u4_t iTOW;      // ms - GPS Time of week of the navigation epoch.
  u1_t version;     // message version (0x00 for this version)
  u1_t posCorValid;     // position covariance matrix validity flag
  u1_t velCorValid;     // velocity covariance matrix validity flag
  u1_t reserved1[9];     // reserved
  r4_t posCovNN;     // m^2 - Position covariance matric value p_NN
  r4_t posCovNE;     // m^2 - Position covariance matric value p_NE
  r4_t posCovND;     // m^2 - Position covariance matric value p_ND
  r4_t posCovEE;     // m^2 - Position covariance matric value p_EE
  r4_t posCovED;     // m^2 - Position covariance matric value p_ED
  r4_t posCovDD;     // m^2 - Position covariance matric value p_DD
  r4_t velCovNN;     // m^2/s^2 - Velocity covariance matric value v_NN
  r4_t velCovNE;     // m^2/s^2 - Velocity covariance matric value v_NE
  r4_t velCovND;     // m^2/s^2 - Velocity covariance matric value v_ND
  r4_t velCovEE;     // m^2/s^2 - Velocity covariance matric value v_EE
  r4_t velCovED;     // m^2/s^2 - Velocity covariance matric value v_ED
  r4_t velCovDD;     // m^2/s^2 - Velocity covariance matric value v_DD

public:
  NavCovPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  NavCovPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    iTOW = buf_offset<u4_t>(&payload_, 0);
    version = buf_offset<u1_t>(&payload_, 4);
    posCorValid = buf_offset<u1_t>(&payload_, 5);
    velCorValid = buf_offset<u1_t>(&payload_, 6);
    posCovNN = buf_offset<r4_t>(&payload_, 16);
    posCovNE = buf_offset<r4_t>(&payload_, 20);
    posCovND = buf_offset<r4_t>(&payload_, 24);
    posCovEE = buf_offset<r4_t>(&payload_, 28);
    posCovED = buf_offset<r4_t>(&payload_, 32);
    posCovDD = buf_offset<r4_t>(&payload_, 36);
    velCovNN = buf_offset<r4_t>(&payload_, 40);
    velCovNE = buf_offset<r4_t>(&payload_, 44);
    velCovND = buf_offset<r4_t>(&payload_, 48);
    velCovEE = buf_offset<r4_t>(&payload_, 52);
    velCovED = buf_offset<r4_t>(&payload_, 56);
    velCovDD = buf_offset<r4_t>(&payload_, 60);
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
    oss << " ver: " << +version;
    oss << " posCorValid: " << +posCorValid;
    oss << " velCorValid: " << +velCorValid;
    oss << std::setprecision(3);
    oss << " posCovNN: " << +posCovNN;
    oss << " posCovNE: " << +posCovNE;
    oss << " posCovND: " << +posCovND;
    oss << " posCovEE: " << +posCovEE;
    oss << " posCovED: " << +posCovED;
    oss << " posCovDD: " << +posCovDD;
    oss << " velCovNN: " << +velCovNN;
    oss << " velCovNE: " << +velCovNE;
    oss << " velCovND: " << +velCovND;
    oss << " velCovEE: " << +velCovEE;
    oss << " velCovED: " << +velCovED;
    oss << " velCovDD: " << +velCovDD;
    return oss.str();
  }
};
}  // namespace ubx::nav::cov
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_COV_HPP_
