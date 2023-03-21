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

#ifndef UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_RTCM_HPP_
#define UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_RTCM_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::rxm::rtcm
{

enum msg_used_t : u1_t {rtcm_used = 2, rtcm_failed = 1, rtcm_unknown = 0};

struct flags_t
{
  union {
    x1_t all;
    struct
    {
      l_t crcFailed : 1;
      msg_used_t msgUsed : 2;
    } bits;
  };
};

class RxmRTCMPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_RXM;
  static const msg_id_t MSG_ID = UBX_RXM_RTCM;

  u1_t version;     // message version (0x02 for this version)
  flags_t flags;    // RTCM input status flags
  u2_t subType;     // only applicable to u-blox proprietary RTCM message 4072
  u2_t refStation;  // reference station id
  u2_t msgType;     // message type

public:
  RxmRTCMPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  RxmRTCMPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    version = buf_offset<u1_t>(&payload_, 0);
    flags = buf_offset<flags_t>(&payload_, 1);
    subType = buf_offset<u2_t>(&payload_, 2);
    refStation = buf_offset<u2_t>(&payload_, 4);
    msgType = buf_offset<u2_t>(&payload_, 6);
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
    oss << " crcFailed: " << +flags.bits.crcFailed;
    oss << " msgUsed: " << +flags.bits.msgUsed;
    oss << " subType: " << +subType;
    oss << " refStation: " << +refStation;
    oss << " msgType: " << +msgType;
    return oss.str();
  }
};
}  // namespace ubx::rxm::rtcm
#endif  // UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_RTCM_HPP_
