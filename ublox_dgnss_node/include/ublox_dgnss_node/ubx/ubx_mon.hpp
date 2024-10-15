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
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/mon/ubx_mon_ver.hpp"
#include "ublox_dgnss_node/ubx/mon/ubx_mon_comms.hpp"

namespace ubx::mon
{

typedef UBXFrameComms<mon::ver::MonVerPayload, usb::Connection> UbxMonVerFrameComms;
typedef UBXFrameComms<mon::comms::MonCommsPayload, usb::Connection> UbxMonCommsFrameComms;

class UbxMon
{
private:
  std::shared_ptr<usb::Connection> usbc_;
  std::shared_ptr<UbxMonVerFrameComms> ver_;
  std::shared_ptr<UbxMonCommsFrameComms> comms_;

public:
  explicit UbxMon(std::shared_ptr<usb::Connection> usbc)
  {
    usbc_ = usbc;
    ver_ = std::make_shared<UbxMonVerFrameComms>(usbc_);
    comms_ = std::make_shared<UbxMonCommsFrameComms>(usbc_);
  }
  std::shared_ptr<UbxMonVerFrameComms> ver()
  {
    return ver_;
  }
  std::shared_ptr<UbxMonCommsFrameComms> comms()
  {
    return comms_;
  }
  void frame(std::shared_ptr<ubx::Frame> frame)
  {
    switch (frame->msg_id) {
      case UBX_MON_VER:
        ver_->frame(frame);
        break;
      case UBX_MON_COMMS:
        comms_->frame(frame);
        break;
      default:
        break;
    }
  }
};
}  // namespace ubx::mon
#endif  // UBLOX_DGNSS_NODE__UBX__UBX_MON_HPP_
