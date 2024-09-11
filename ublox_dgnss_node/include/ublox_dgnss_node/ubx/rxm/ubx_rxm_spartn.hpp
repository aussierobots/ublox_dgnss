// Copyright 2024 Australian Robotics Supplies & Technology
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

#ifndef UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_SPARTN_HPP_
#define UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_SPARTN_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::rxm::spartn
{

// Define enum for msgUsed in the flags field
enum msg_used_t : u1_t
{
  msg_unknown = 0,
  msg_not_used = 1,
  msg_used = 2
};

struct RxmSpartnPayload : public UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_RXM;
  static const msg_id_t MSG_ID = UBX_RXM_SPARTN;

  u1_t version;
  msg_used_t msg_used;  // Extracted from bits 2...1 of flags
  u2_t subType;
  u1_t reserved0[2];
  u2_t msgType;

public:
  RxmSpartnPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  RxmSpartnPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    version = buf_offset<u1_t>(&payload_, 0);

    // Extract bits 2...1 of flags to determine msgUsed
    u1_t flags = buf_offset<u1_t>(&payload_, 1);
    msg_used = static_cast<msg_used_t>((flags >> 1) & 0x03);

    subType = buf_offset<u2_t>(&payload_, 2);
    reserved0[0] = buf_offset<u1_t>(&payload_, 4);
    reserved0[1] = buf_offset<u1_t>(&payload_, 5);
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
    oss << " msg_used: " << +static_cast<u1_t>(msg_used);
    oss << " subType: " << subType;
    oss << " msgType: " << msgType;

    return oss.str();
  }
};

}  // namespace ubx::rxm::spartn

#endif  // UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_SPARTN_HPP_
