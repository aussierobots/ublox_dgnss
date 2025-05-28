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

#ifndef UBLOX_DGNSS_NODE__UBX__MON__UBX_MON_COMMS_HPP_
#define UBLOX_DGNSS_NODE__UBX__MON__UBX_MON_COMMS_HPP_

#include "ublox_dgnss_node/ubx/ubx.hpp"
#include <cstring>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace ubx::mon::comms
{

struct PortInfo
{
  u2_t portId;
  u2_t txPending;
  u4_t txBytes;
  u1_t txUsage;
  u1_t txPeakUsage;
  u2_t rxPending;
  u4_t rxBytes;
  u1_t rxUsage;
  u1_t rxPeakUsage;
  u2_t overrunErrs;
  u2_t msgs[4];
  u4_t skipped;
};

class MonCommsPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_MON;
  static const msg_id_t MSG_ID = UBX_MON_COMMS;

  u1_t version;
  u1_t nPorts;
  x1_t txErrors;
  u1_t reserved0;
  u1_t protIds[4];
  std::vector<PortInfo> ports;

public:
  MonCommsPayload()
  : UBXPayload(MSG_CLASS, MSG_ID) {}

  MonCommsPayload(u1_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    auto ptr = payload_.data();
    memcpy(&version, ptr, sizeof(version));
    ptr += sizeof(version);
    memcpy(&nPorts, ptr, sizeof(nPorts));
    ptr += sizeof(nPorts);
    memcpy(&txErrors, ptr, sizeof(txErrors));
    ptr += sizeof(txErrors);
    ptr += sizeof(reserved0);   // skip reserved0
    memcpy(protIds, ptr, sizeof(protIds));
    ptr += sizeof(protIds);

    // Manually parse each PortInfo field to avoid issues with struct packing/alignment
    for (int i = 0; i < nPorts; ++i) {
      PortInfo info;
      memcpy(&info.portId, ptr, 2); ptr += 2;
      memcpy(&info.txPending, ptr, 2); ptr += 2;
      memcpy(&info.txBytes, ptr, 4); ptr += 4;
      memcpy(&info.txUsage, ptr, 1); ptr += 1;
      memcpy(&info.txPeakUsage, ptr, 1); ptr += 1;
      memcpy(&info.rxPending, ptr, 2); ptr += 2;
      memcpy(&info.rxBytes, ptr, 4); ptr += 4;
      memcpy(&info.rxUsage, ptr, 1); ptr += 1;
      memcpy(&info.rxPeakUsage, ptr, 1); ptr += 1;
      memcpy(&info.overrunErrs, ptr, 2); ptr += 2;
      memcpy(&info.msgs, ptr, 8); ptr += 8; // 4 x u2_t
      ptr += 8; // skip reserved1[8]
      memcpy(&info.skipped, ptr, 4); ptr += 4;
      ports.push_back(info);
    }
  }

  std::tuple<u1_t *, size_t> make_poll_payload() override
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }

  std::string to_string()
  {
    std::ostringstream oss;
    oss << "version: " << static_cast<int>(version);
    oss << " nPorts: " << static_cast<int>(nPorts);
    oss << " txErrors: " << static_cast<int>(txErrors);
    for (auto & port : ports) {
      oss << " PortID: " << port.portId << " TxPending: " << port.txPending << " TxBytes: " <<
        port.txBytes;
    }
    return oss.str();
  }
};

}  // namespace ubx::mon::comms

#endif  // UBLOX_DGNSS_NODE__UBX__MON__UBX_MON_COMMS_HPP_
