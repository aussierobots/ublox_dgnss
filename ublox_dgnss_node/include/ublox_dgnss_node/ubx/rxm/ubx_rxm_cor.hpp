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

#ifndef UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_COR_HPP_
#define UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_COR_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::rxm::cor
{

// Enum for protocol in statusInfo
enum protocol_t : u1_t
{
  protocol_unknown = 0,
  protocol_rtcm3 = 1,
  protocol_spartn = 2,
  protocol_ubx_rxm_pmp = 29,
  protocol_ubx_rxm_qzssl6 = 30
};

// Enum for errStatus in statusInfo
enum err_status_t : u1_t
{
  err_unknown = 0,
  err_error_free = 1,
  err_erroneous = 2
};

// Enum for msgUsed in statusInfo
enum msg_used_t : u1_t
{
  msg_unknown = 0,
  msg_not_used = 1,
  msg_used = 2
};

// Enum for msgEncrypted in statusInfo
enum msg_encrypted_t : u1_t
{
  msg_encryption_unknown = 0,
  msg_not_encrypted = 1,
  msg_encrypted = 2
};

// Enum for msgDecrypted in statusInfo
enum msg_decrypted_t : u1_t
{
  msg_decryption_unknown = 0,
  msg_not_decrypted = 1,
  msg_decrypted = 2
};

class RxmCorPayload : public UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_RXM;
  static const msg_id_t MSG_ID = UBX_RXM_COR;

  u1_t version;
  u1_t ebno;
  u1_t reserved0[2];
  u4_t statusInfo;
  u2_t msgType;
  u2_t msgSubType;

  protocol_t protocol;
  err_status_t errStatus;
  msg_used_t msgUsed;
  u2_t correctionId;  // Corrected to u2_t, as it's a 16-bit value
  bool msgTypeValid;
  bool msgSubTypeValid;
  bool msgInputHandle;
  msg_encrypted_t msgEncrypted;
  msg_decrypted_t msgDecrypted;

public:
  RxmCorPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  RxmCorPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    version = buf_offset<u1_t>(&payload_, 0);
    ebno = buf_offset<u1_t>(&payload_, 1);
    reserved0[0] = buf_offset<u1_t>(&payload_, 2);
    reserved0[1] = buf_offset<u1_t>(&payload_, 3);

    statusInfo = buf_offset<u4_t>(&payload_, 4);

    protocol = static_cast<protocol_t>(statusInfo & 0x1F);  // bits 4...0
    errStatus = static_cast<err_status_t>((statusInfo >> 5) & 0x03);  // bits 6...5
    msgUsed = static_cast<msg_used_t>((statusInfo >> 7) & 0x03);  // bits 8...7
    correctionId = static_cast<u2_t>((statusInfo >> 9) & 0xFFFF);  // bits 24...9, updated to u2_t
    msgTypeValid = (statusInfo >> 25) & 0x01;  // bit 25
    msgSubTypeValid = (statusInfo >> 26) & 0x01;  // bit 26
    msgInputHandle = (statusInfo >> 27) & 0x01;  // bit 27
    msgEncrypted = static_cast<msg_encrypted_t>((statusInfo >> 28) & 0x03);  // bits 29...28
    msgDecrypted = static_cast<msg_decrypted_t>((statusInfo >> 30) & 0x03);  // bits 31...30

    msgType = buf_offset<u2_t>(&payload_, 8);
    msgSubType = buf_offset<u2_t>(&payload_, 10);
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
    oss << " ebno: " << +ebno * std::pow(2, -3);
    oss << " protocol: " << +static_cast<u1_t>(protocol);
    oss << " errStatus: " << +static_cast<u1_t>(errStatus);
    oss << " msgUsed: " << +static_cast<u1_t>(msgUsed);
    oss << " correctionId: " << correctionId;
    oss << " msgTypeValid: " << std::boolalpha << msgTypeValid;
    oss << " msgSubTypeValid: " << std::boolalpha << msgSubTypeValid;
    oss << " msgInputHandle: " << std::boolalpha << msgInputHandle;
    oss << " msgEncrypted: " << +static_cast<u1_t>(msgEncrypted);
    oss << " msgDecrypted: " << +static_cast<u1_t>(msgDecrypted);
    oss << " msgType: " << msgType;
    oss << " msgSubType: " << msgSubType;

    return oss.str();
  }
};

}  // namespace ubx::rxm::cor

#endif  // UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_COR_HPP_
