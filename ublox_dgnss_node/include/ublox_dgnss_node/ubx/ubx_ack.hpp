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

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_ACK_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_ACK_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/usb.hpp"

namespace ubx::ack
{
class AckPayload : UBXPayload
{
public:
  msg_class_t msg_class;
  msg_id_t msg_id;

public:
  AckPayload()
  {
    msg_class = 0;
    msg_id = 0;
  }
  AckPayload(ch_t * payload, u2_t size)
  {
    if (size == 2) {
      msg_class = payload[0];
      msg_id = payload[1];
    }
  }
  std::tuple<const u1_t *, size_t> Payload()
  {
    payload_.clear();
    payload_.push_back(msg_class);
    payload_.push_back(msg_id);
    return std::make_tuple(payload_.data(), payload_.size());
  }

  std::string to_string()
  {
    std::ostringstream os;
    os << "class: 0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << +msg_class;
    os << " id: 0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << +msg_id;
    return os.str();
  }
};

class AckAckPayload : public AckPayload, public UBXPayloadOutputPrint
{
public:
  AckAckPayload(ch_t * payload, u2_t size)
  {
    if (size == 2) {
      msg_class = payload[0];
      msg_id = payload[1];
    }
  }
};
class AckNakPayload : public AckPayload, public UBXPayloadOutputPrint
{
public:
  AckNakPayload(ch_t * payload, u2_t size)
  {
    if (size == 2) {
      msg_class = payload[0];
      msg_id = payload[1];
    }
  }
};
}  // namespace ubx::ack

#endif  // UBLOX_DGNSS_NODE__UBX__UBX_ACK_HPP_
