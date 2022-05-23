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

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_MON_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_MON_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"

namespace ubx::mon
{

class MonVerPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_MON;
  static const msg_id_t MSG_ID = UBX_MON_VER;

  ch_t sw_version[30];        // null-terminated software version string
  ch_t hw_version[10];        // null-terminated hardware version string
  std::vector<std::string> extension;     // extended software information strings

public:
  MonVerPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  MonVerPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    auto ptr = payload_.data();

    memcpy(&sw_version, ptr, sizeof(sw_version) );
    ptr += 30;
    memcpy(&hw_version, ptr, sizeof(hw_version) );
    ptr += 10;

    while (ptr < (payload_.data() + size)) {
      std::string s1(reinterpret_cast<char *>(ptr));
      extension.push_back(s1);
      ptr += 30;
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
    oss << "sw_version: " << sw_version;
    oss << " hw_version: " << hw_version;
    for (auto e : extension) {
      oss << " " << e;
    }
    return oss.str();
  }
};

typedef UBXFrameComms<mon::MonVerPayload, usb::Connection> UbxMonVerFrameComms;

class UbxMon
{
private:
  std::shared_ptr<usb::Connection> usbc_;
  std::shared_ptr<UbxMonVerFrameComms> ver_;

public:
  explicit UbxMon(std::shared_ptr<usb::Connection> usbc)
  {
    usbc_ = usbc;
    ver_ = std::make_shared<UbxMonVerFrameComms>(usbc_);
  }
  std::shared_ptr<UbxMonVerFrameComms> ver()
  {
    return ver_;
  }
};
}  // namespace ubx::mon
#endif  // UBLOX_DGNSS_NODE__UBX__UBX_MON_HPP_
