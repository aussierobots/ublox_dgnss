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

#ifndef UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_SPARTN_KEY_HPP_
#define UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_SPARTN_KEY_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::rxm::spartnkey
{

// Struct for the first repeated group (key information)
struct KeyInfo
{
  u1_t reserved1;
  u1_t keyLengthBytes;
  u2_t validFromWno;
  u4_t validFromTow;

  KeyInfo() = default;

  KeyInfo(std::vector<u1_t> * payload, size_t offset)
  {
    reserved1 = buf_offset<u1_t>(payload, offset);
    keyLengthBytes = buf_offset<u1_t>(payload, offset + 1);
    validFromWno = buf_offset<u2_t>(payload, offset + 2);
    validFromTow = buf_offset<u4_t>(payload, offset + 4);
  }
};

// Struct for the second repeated group (key payload)
struct KeyPayload
{
  std::vector<u1_t> key;

  KeyPayload() = default;

  KeyPayload(std::vector<u1_t> * payload, size_t offset, u1_t keyLength)
  {
    key.resize(keyLength);
    memcpy(key.data(), payload->data() + offset, keyLength);
  }
};

class RxmSpartnKeyPayload : public UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_RXM;
  static const msg_id_t MSG_ID = UBX_RXM_SPARTNKEY;

  u1_t version;
  u1_t numKeys;
  u1_t reserved0[2];
  std::vector<KeyInfo> keyInfos;
  std::vector<KeyPayload> keyPayloads;

public:
  RxmSpartnKeyPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  RxmSpartnKeyPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    version = buf_offset<u1_t>(&payload_, 0);
    numKeys = buf_offset<u1_t>(&payload_, 1);
    reserved0[0] = buf_offset<u1_t>(&payload_, 2);
    reserved0[1] = buf_offset<u1_t>(&payload_, 3);

// Process each key info block (numKeys times)
    size_t offset = 4;
    keyInfos.clear();
    for (u1_t i = 0; i < numKeys; ++i) {
      keyInfos.emplace_back(&payload_, offset);  // Use constructor
      offset += 8;  // Each key info is 8 bytes
    }

    // Process the key payloads (lengths defined in keyLengthBytes)
    keyPayloads.clear();
    for (u1_t i = 0; i < numKeys; ++i) {
      keyPayloads.emplace_back(&payload_, offset, keyInfos[i].keyLengthBytes);  // Use constructor
      offset += keyInfos[i].keyLengthBytes;
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
    oss << "version: " << +version;
    oss << " numKeys: " << +numKeys;

    for (size_t i = 0; i < keyInfos.size(); ++i) {
      oss << "\n  Key " << i + 1 << ": ";
      oss << "reserved1: " << +keyInfos[i].reserved1;
      oss << " keyLengthBytes: " << +keyInfos[i].keyLengthBytes;
      oss << " validFromWno: " << keyInfos[i].validFromWno;
      oss << " validFromTow: " << keyInfos[i].validFromTow;
    }

    for (size_t i = 0; i < keyPayloads.size(); ++i) {
      oss << "\n  Key Payload " << i + 1 << ": ";
      oss << "key: ";
      for (size_t j = 0; j < keyPayloads[i].key.size(); ++j) {
        oss << std::hex << +keyPayloads[i].key[j] << " ";
      }
    }

    return oss.str();
  }
};

}  // namespace ubx::rxm::spartnkey

#endif  // UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_SPARTN_KEY_HPP_
