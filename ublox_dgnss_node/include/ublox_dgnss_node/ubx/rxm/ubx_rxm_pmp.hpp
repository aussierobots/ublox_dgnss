// Copyright 2026 Australian Robotics Supplies & Technology
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

#ifndef UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_PMP_HPP_
#define UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_PMP_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"
#include "ublox_ubx_msgs/msg/ubx_rxm_pmp.hpp"

namespace ubx::rxm::pmp
{

// UBX-RXM-PMP (0x02 0x72) - Point to Multipoint (L-band) input message.
// Type: Input on the X20/F9 receiver - the host sends L-band data obtained from a
// NEO-D9S companion module to the device, so this payload is serialized and written
// to USB (it is not polled/output by the receiver).
class RxmPmpPayload : public UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_RXM;
  static const msg_id_t MSG_ID = UBX_RXM_PMP;

  u1_t version;
  u1_t reserved0;
  u2_t num_bytes_user_data;       // number of bytes in userData this frame (0..504)
  u4_t time_tag;                  // ms - time since startup when frame started
  u4_t unique_word[2];            // received unique words
  u2_t service_identifier;        // received service identifier
  u1_t spare;                     // received spare data
  u1_t unique_word_bit_errors;    // number of bit errors in both unique words
  u2_t fec_bits;                  // number of bits corrected by FEC
  u1_t ebno;                      // 2^-3 dB - energy per bit to noise power spectral density
  u1_t reserved1;
  std::vector<u1_t> user_data;    // received user data (= num_bytes_user_data)

public:
  RxmPmpPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  // Load the payload fields from a ROS message prior to sending to the device
  void load_from_msg(const ublox_ubx_msgs::msg::UBXRxmPmp & msg)
  {
    version = msg.version;
    reserved0 = 0;
    time_tag = msg.time_tag;
    unique_word[0] = msg.unique_word[0];
    unique_word[1] = msg.unique_word[1];
    service_identifier = msg.service_identifier;
    spare = msg.spare;
    unique_word_bit_errors = msg.unique_word_bit_errors;
    fec_bits = msg.fec_bits;
    ebno = msg.ebno;
    reserved1 = 0;

    user_data.assign(msg.user_data.begin(), msg.user_data.end());

    // use the declared length if provided, otherwise the array size
    num_bytes_user_data = msg.num_bytes_user_data;
    if (num_bytes_user_data == 0) {
      num_bytes_user_data = static_cast<u2_t>(user_data.size());
    }
  }

  // Serialize the payload bytes for sending to the device
  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();

    payload_.push_back(version);
    payload_.push_back(reserved0);
    buf_append_u2(&payload_, num_bytes_user_data);
    buf_append_u4(&payload_, time_tag);
    buf_append_u4(&payload_, unique_word[0]);
    buf_append_u4(&payload_, unique_word[1]);
    buf_append_u2(&payload_, service_identifier);
    payload_.push_back(spare);
    payload_.push_back(unique_word_bit_errors);
    buf_append_u2(&payload_, fec_bits);
    payload_.push_back(ebno);
    payload_.push_back(reserved1);

    for (const auto & byte : user_data) {
      payload_.push_back(byte);
    }

    return std::make_tuple(payload_.data(), payload_.size());
  }

  std::string to_string()
  {
    std::ostringstream oss;
    oss << "version: " << +version;
    oss << ", num_bytes_user_data: " << num_bytes_user_data;
    oss << ", time_tag: " << time_tag;
    oss << ", unique_word: 0x" << std::hex << unique_word[1] << unique_word[0] << std::dec;
    oss << ", service_identifier: " << service_identifier;
    oss << ", unique_word_bit_errors: " << +unique_word_bit_errors;
    oss << ", fec_bits: " << fec_bits;
    oss << ", ebno: " << +ebno * std::pow(2, -3);
    return oss.str();
  }
};
}  // namespace ubx::rxm::pmp

#endif  // UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_PMP_HPP_
