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

#ifndef UBLOX_DGNSS_NODE__UBX__SEC__UBX_SEC_SIG_HPP_
#define UBLOX_DGNSS_NODE__UBX__SEC__UBX_SEC_SIG_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::sec::sig
{

struct jam_flags_t
{
  union {
    x1_t all;
    struct
    {
      u1_t jam_det_enabled : 1;
      u1_t jamming_state : 2;
      u1_t reserved : 5;
    } bits;
  };
};

struct spf_flags_t
{
  union {
    x1_t all;
    struct
    {
      u1_t spf_det_enabled : 1;
      u1_t spoofing_state : 3;
      u1_t reserved : 4;
    } bits;
  };
};

class SecSigPayload : public UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_SEC;
  static const msg_id_t MSG_ID = UBX_SEC_SIG;

  u1_t version;
  jam_flags_t jam_flags;
  spf_flags_t spf_flags;

public:
  SecSigPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  SecSigPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    version = buf_offset<u1_t>(&payload_, 0);
    jam_flags.all = buf_offset<x1_t>(&payload_, 4);
    spf_flags.all = buf_offset<x1_t>(&payload_, 8);
  }

  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }

  std::string to_string()
  {
    std::ostringstream oss;
    oss << "version: " << static_cast<int>(version);
    oss << ", jam_flags: {jam_det_enabled: " << static_cast<int>(jam_flags.bits.jam_det_enabled);
    oss << ", jamming_state: " << static_cast<int>(jam_flags.bits.jamming_state) << "}";
    oss << ", spf_flags: {spf_det_enabled: " << static_cast<int>(spf_flags.bits.spf_det_enabled);
    oss << ", spoofing_state: " << static_cast<int>(spf_flags.bits.spoofing_state) << "}";
    return oss.str();
  }
};

}  // namespace ubx::sec::sig

#endif  // UBLOX_DGNSS_NODE__UBX__SEC__UBX_SEC_SIG_HPP_
