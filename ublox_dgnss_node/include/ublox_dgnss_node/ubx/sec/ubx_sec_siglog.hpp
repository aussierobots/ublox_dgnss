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

#ifndef UBLOX_DGNSS_NODE__UBX__SEC__UBX_SEC_SIGLOG_HPP_
#define UBLOX_DGNSS_NODE__UBX__SEC__UBX_SEC_SIGLOG_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::sec::siglog
{

struct SigLogEvent_t
{
  u4_t time_elapsed;
  u1_t detection_type;
  u1_t event_type;
};

class SecSigLogPayload : public UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_SEC;
  static const msg_id_t MSG_ID = UBX_SEC_SIGLOG;

  u1_t version;
  u1_t num_events;
  std::vector<SigLogEvent_t> events;

public:
  SecSigLogPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  SecSigLogPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    version = buf_offset<u1_t>(&payload_, 0);
    num_events = buf_offset<u1_t>(&payload_, 1);

    events.clear();
    size_t offset = 8;
    for (u1_t i = 0; i < num_events; ++i) {
      SigLogEvent_t event;
      event.time_elapsed = buf_offset<u4_t>(&payload_, offset);
      event.detection_type = buf_offset<u1_t>(&payload_, offset + 4);
      event.event_type = buf_offset<u1_t>(&payload_, offset + 5);

      events.push_back(event);
      offset += 8;
    }
  }

  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }

  std::string to_string()
  {
    std::ostringstream oss;
    oss << "version: " << static_cast<int>(version) << ", num_events: " <<
      static_cast<int>(num_events);

    for (size_t i = 0; i < events.size(); ++i) {
      oss << "\n  event " << i << ": ";
      oss << "time_elapsed: " << events[i].time_elapsed << ", detection_type: " <<
        static_cast<int>(events[i].detection_type);
      oss << ", event_type: " << static_cast<int>(events[i].event_type);
    }

    return oss.str();
  }
};

}  // namespace ubx::sec::siglog

#endif  // UBLOX_DGNSS_NODE__UBX__SEC__UBX_SEC_SIGLOG_HPP_
