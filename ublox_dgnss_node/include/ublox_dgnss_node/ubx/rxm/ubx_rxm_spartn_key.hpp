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
#include "ublox_ubx_msgs/msg/ubx_rxm_spartn_key.hpp"

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

// Input variant of UBX-RXM-SPARTNKEY used to push dynamic SPARTN decryption keys
// to the device. Mirrors the ESFMeasFullPayload pattern: load_from_msg() maps a ROS
// message into the fields and make_poll_payload() serializes them for sending to USB.
class RxmSpartnKeyFullPayload : public UBXPayload
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
  RxmSpartnKeyFullPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  // Load the payload fields from a ROS message prior to sending to the device
  void load_from_msg(const ublox_ubx_msgs::msg::UBXRxmSpartnKey & msg)
  {
    version = msg.version;
    reserved0[0] = 0;
    reserved0[1] = 0;

    keyInfos.clear();
    for (const auto & ki : msg.key_info) {
      KeyInfo key_info;
      key_info.reserved1 = ki.reserved1;
      key_info.keyLengthBytes = ki.key_length_bytes;
      key_info.validFromWno = ki.valid_from_wno;
      key_info.validFromTow = ki.valid_from_tow;
      keyInfos.push_back(key_info);
    }

    // use the declared key count if provided, otherwise the key_info size
    numKeys = msg.num_keys;
    if (numKeys == 0) {
      numKeys = static_cast<u1_t>(keyInfos.size());
    }

    // split the concatenated key_payload back into per-key payloads using each
    // key's keyLengthBytes
    keyPayloads.clear();
    size_t offset = 0;
    for (const auto & key_info : keyInfos) {
      KeyPayload key_payload;
      for (u1_t j = 0; j < key_info.keyLengthBytes && offset < msg.key_payload.size(); ++j) {
        key_payload.key.push_back(msg.key_payload[offset]);
        ++offset;
      }
      keyPayloads.push_back(key_payload);
    }
  }

  // Serialize the payload bytes for sending to the device
  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();

    payload_.push_back(version);
    payload_.push_back(numKeys);
    payload_.push_back(reserved0[0]);
    payload_.push_back(reserved0[1]);

    for (const auto & key_info : keyInfos) {
      payload_.push_back(key_info.reserved1);
      payload_.push_back(key_info.keyLengthBytes);
      buf_append_u2(&payload_, key_info.validFromWno);
      buf_append_u4(&payload_, key_info.validFromTow);
    }

    for (const auto & key_payload : keyPayloads) {
      for (const auto & byte : key_payload.key) {
        payload_.push_back(byte);
      }
    }

    return std::make_tuple(payload_.data(), payload_.size());
  }

  std::string to_string()
  {
    std::ostringstream oss;
    oss << "version: " << +version;
    oss << " numKeys: " << +numKeys;
    for (size_t i = 0; i < keyInfos.size(); ++i) {
      oss << "\n  Key " << i + 1 << ": ";
      oss << "keyLengthBytes: " << +keyInfos[i].keyLengthBytes;
      oss << " validFromWno: " << keyInfos[i].validFromWno;
      oss << " validFromTow: " << keyInfos[i].validFromTow;
    }
    return oss.str();
  }
};

}  // namespace ubx::rxm::spartnkey

#endif  // UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_SPARTN_KEY_HPP_
