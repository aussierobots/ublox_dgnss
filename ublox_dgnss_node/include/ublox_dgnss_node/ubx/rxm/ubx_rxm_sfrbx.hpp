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

#ifndef UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_SFRBX_HPP_
#define UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_SFRBX_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::rxm::sfrbx
{

class RxmSfrbxPayload : public UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_RXM;
  static const msg_id_t MSG_ID = UBX_RXM_SFRBX;

  u1_t gnss_id;
  u1_t sv_id;
  u1_t sig_id;
  u1_t freq_id;
  u1_t num_words;
  u1_t chn;
  u1_t version;
  std::vector<u4_t> dwrd;

public:
  RxmSfrbxPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  RxmSfrbxPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    gnss_id = buf_offset<u1_t>(&payload_, 0);
    sv_id = buf_offset<u1_t>(&payload_, 1);
    sig_id = buf_offset<u1_t>(&payload_, 2);
    freq_id = buf_offset<u1_t>(&payload_, 3);
    num_words = buf_offset<u1_t>(&payload_, 4);
    chn = buf_offset<u1_t>(&payload_, 5);
    version = buf_offset<u1_t>(&payload_, 6);

    dwrd.clear();
    size_t offset = 8;
    for (u1_t i = 0; i < num_words; ++i) {
      dwrd.push_back(buf_offset<u4_t>(&payload_, offset));
      offset += 4;
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
    oss << "gnss_id: " << static_cast<int>(gnss_id);
    oss << ", sv_id: " << static_cast<int>(sv_id);
    oss << ", sig_id: " << static_cast<int>(sig_id);
    oss << ", freq_id: " << static_cast<int>(freq_id);
    oss << ", num_words: " << static_cast<int>(num_words);
    oss << ", chn: " << static_cast<int>(chn);
    oss << ", version: " << static_cast<int>(version);

    for (size_t i = 0; i < dwrd.size(); ++i) {
      oss << ", dwrd[" << i << "]: 0x" << std::hex << dwrd[i] << std::dec;
    }

    return oss.str();
  }
};
}  // namespace ubx::rxm::sfrbx

#endif  // UBLOX_DGNSS_NODE__UBX__RXM__UBX_RXM_SFRBX_HPP_
