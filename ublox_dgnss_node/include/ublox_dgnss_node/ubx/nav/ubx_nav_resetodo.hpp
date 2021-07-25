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

#ifndef UBLOX_DGNSS_NODE__UBX_NAV_RESETODO_HPP
#define UBLOX_DGNSS_NODE__UBX_NAV_RESETODO_HPP

#include <unistd.h>
#include <memory>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include <tuple>
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx {
  namespace nav::resetodo {

   class NavResetOdoPayload : UBXPayload {
      public:
      const static msg_class_t MSG_CLASS = UBX_NAV;
      const static msg_id_t MSG_ID = UBX_NAV_RESETODO;

      // no payload for a command

      public:
      NavResetOdoPayload() : UBXPayload(MSG_CLASS,MSG_ID) {}
      NavResetOdoPayload(ch_t* payload_polled, u2_t size) : UBXPayload(MSG_CLASS,MSG_ID) {
        payload_.clear();
        payload_.reserve(size);
        payload_.resize(size);
        memcpy(payload_.data(), payload_polled, size);
      }
      std::tuple<const u1_t*, size_t> make_poll_payload() {
        payload_.clear();
        return std::make_tuple(payload_.data(), payload_.size());
      }
      std::string to_string() {
        std::ostringstream oss;
        oss << " command only - no payload";
        return oss.str();
      }
    };
  }
}
#endif // UBLOX_DGNSS_NODE__UBX_NAV_RESETODO_HPP