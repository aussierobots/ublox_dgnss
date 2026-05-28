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

#ifndef UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_QZSSL6_HPP_
#define UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_QZSSL6_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"
#include "ublox_ubx_msgs/msg/ubx_rxm_qzssl6.hpp"

namespace ubx::rxm::qzssl6
{

// Message name in chInfo bit 10
enum msg_name_t : u1_t
{
  msg_name_l6d = 0,
  msg_name_l6e = 1
};

// Error status in chInfo bits 13...12
enum err_status_t : u1_t
{
  err_unknown = 0,
  err_error_free = 1,
  err_erroneous = 2
};

// Channel name in chInfo bits 15...14
enum ch_name_t : u1_t
{
  ch_name_a = 0,
  ch_name_b = 1
};

// UBX-RXM-QZSSL6 (0x02 0x73) - QZSS L6 (CLAS) input message.
// Type: Input on the X20/F9 receiver - the host sends QZSS L6 data obtained from a
// NEO-D9C companion module to the device, so this payload is serialized and written
// to USB (it is not polled/output by the receiver).
class RxmQzssl6Payload : public UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_RXM;
  static const msg_id_t MSG_ID = UBX_RXM_QZSSL6;

  u1_t version;
  u1_t sv_id;
  u2_t cno;                 // 2^-8 dBHz - mean C/N0
  u4_t time_tag;            // ms - local time tag at start of received QZSS L6 message
  u1_t group_delay;         // ns - L6 group delay w.r.t. L2 on channel
  u1_t bit_err_corr;        // bit errors corrected by Reed-Solomon decoder
  x2_t ch_info;             // receiver channel info bitfield

  u1_t chn;                 // chInfo bits 9...8 - receiver channel (0, 1)
  msg_name_t msg_name;      // chInfo bit 10 - 0=L6D, 1=L6E
  err_status_t err_status;  // chInfo bits 13...12
  ch_name_t ch_name;        // chInfo bits 15...14 - 0=channel A, 1=channel B

  u1_t reserved0[2];
  std::vector<u1_t> msg_bytes;  // bytes in a QZSS L6 message (250 bytes)

public:
  RxmQzssl6Payload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  // Load the payload fields from a ROS message prior to sending to the device
  void load_from_msg(const ublox_ubx_msgs::msg::UBXRxmQzssl6 & msg)
  {
    version = msg.version;
    sv_id = msg.sv_id;
    cno = msg.cno;
    time_tag = msg.time_tag;
    group_delay = msg.group_delay;
    bit_err_corr = msg.bit_err_corr;

    chn = msg.chn;
    msg_name = static_cast<msg_name_t>(msg.msg_name);
    err_status = static_cast<err_status_t>(msg.err_status);
    ch_name = static_cast<ch_name_t>(msg.ch_name);

    // recombine the decoded fields into the chInfo bitfield
    ch_info = static_cast<x2_t>(
      ((chn & 0x03) << 8) |
      ((static_cast<u1_t>(msg_name) & 0x01) << 10) |
      ((static_cast<u1_t>(err_status) & 0x03) << 12) |
      ((static_cast<u1_t>(ch_name) & 0x03) << 14));

    reserved0[0] = 0;
    reserved0[1] = 0;

    msg_bytes.assign(msg.msg_bytes.begin(), msg.msg_bytes.end());
  }

  // Serialize the payload bytes for sending to the device
  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();

    payload_.push_back(version);
    payload_.push_back(sv_id);
    buf_append_u2(&payload_, cno);
    buf_append_u4(&payload_, time_tag);
    payload_.push_back(group_delay);
    payload_.push_back(bit_err_corr);
    buf_append_x2(&payload_, ch_info);
    payload_.push_back(reserved0[0]);
    payload_.push_back(reserved0[1]);

    for (const auto & byte : msg_bytes) {
      payload_.push_back(byte);
    }

    return std::make_tuple(payload_.data(), payload_.size());
  }

  std::string to_string()
  {
    std::ostringstream oss;
    oss << "version: " << +version;
    oss << ", sv_id: " << +sv_id;
    oss << ", cno: " << +cno * std::pow(2, -8) << " dBHz";
    oss << ", time_tag: " << time_tag;
    oss << ", group_delay: " << +group_delay;
    oss << ", bit_err_corr: " << +bit_err_corr;
    oss << ", chn: " << +chn;
    oss << ", msg_name: " << +static_cast<u1_t>(msg_name);
    oss << ", err_status: " << +static_cast<u1_t>(err_status);
    oss << ", ch_name: " << +static_cast<u1_t>(ch_name);
    oss << ", msg_bytes size: " << msg_bytes.size();
    return oss.str();
  }
};
}  // namespace ubx::rxm::qzssl6

#endif  // UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_QZSSL6_HPP_
