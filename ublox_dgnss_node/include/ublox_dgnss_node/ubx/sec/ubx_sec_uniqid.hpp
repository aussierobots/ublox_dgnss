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

#ifndef UBLOX_DGNSS_NODE__UBX__SEC__UBX_SEC_UNIQID_HPP_
#define UBLOX_DGNSS_NODE__UBX__SEC__UBX_SEC_UNIQID_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::sec::uniqid
{

class SecUniqidPayload : public UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_SEC;
  static const msg_id_t MSG_ID = UBX_SEC_UNIQID;

  u1_t version;
  u1_t unique_id[5];  // Unique chip ID of size 5 bytes

public:
  SecUniqidPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  SecUniqidPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    version = buf_offset<u1_t>(&payload_, 0);
    memcpy(unique_id, &payload_[4], 5);  // Copy 5 bytes of Unique ID
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
    oss << ", unique_id: ";
    for (int i = 0; i < 5; ++i) {
      oss << std::hex << static_cast<int>(unique_id[i]) << " ";
    }
    return oss.str();
  }
};

}  // namespace ubx::sec::uniqid

#endif  // UBLOX_DGNSS_NODE__UBX__SEC__UBX_SEC_UNIQID_HPP_
