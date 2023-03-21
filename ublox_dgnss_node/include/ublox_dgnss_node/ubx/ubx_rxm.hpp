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

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_RXM_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_RXM_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"
#include "ublox_dgnss_node/ubx/rxm/ubx_rxm_rtcm.hpp"

namespace ubx::rxm
{

typedef UBXFrameComms<rxm::rtcm::RxmRTCMPayload, usb::Connection> UbxRxmRTCMFrameComms;

class UbxRxm
{
private:
  std::shared_ptr<usb::Connection> usbc_;
  std::shared_ptr<UbxRxmRTCMFrameComms> rtcm_;

public:
  explicit UbxRxm(std::shared_ptr<usb::Connection> usbc)
  {
    usbc_ = usbc;
    rtcm_ = std::make_shared<UbxRxmRTCMFrameComms>(usbc_);
  }
  std::shared_ptr<UbxRxmRTCMFrameComms> rtcm()
  {
    return rtcm_;
  }
};
}  // namespace ubx::rxm
#endif  // UBLOX_DGNSS_NODE__UBX__UBX_RXM_HPP_
