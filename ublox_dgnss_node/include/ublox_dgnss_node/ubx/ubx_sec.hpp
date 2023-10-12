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

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_SEC_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_SEC_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"
#include "ublox_dgnss_node/ubx/sec/ubx_sec_sig.hpp"
#include "ublox_dgnss_node/ubx/sec/ubx_sec_siglog.hpp"
#include "ublox_dgnss_node/ubx/sec/ubx_sec_uniqid.hpp"

namespace ubx::sec
{

typedef UBXFrameComms<sec::sig::SecSigPayload, usb::Connection> UbxSecSigFrameComms;
typedef UBXFrameComms<sec::siglog::SecSigLogPayload, usb::Connection> UbxSecSigLogFrameComms;
typedef UBXFrameComms<sec::uniqid::SecUniqidPayload, usb::Connection> UbxSecUniqidFrameComms;

class UbxSec
{
private:
  std::shared_ptr<usb::Connection> usbc_;
  std::shared_ptr<UbxSecSigFrameComms> sig_;
  std::shared_ptr<UbxSecSigLogFrameComms> siglog_;
  std::shared_ptr<UbxSecUniqidFrameComms> uniqid_;

public:
  explicit UbxSec(std::shared_ptr<usb::Connection> usbc)
  {
    usbc_ = usbc;
    sig_ = std::make_shared<UbxSecSigFrameComms>(usbc_);
    siglog_ = std::make_shared<UbxSecSigLogFrameComms>(usbc_);
    uniqid_ = std::make_shared<UbxSecUniqidFrameComms>(usbc_);
  }

  std::shared_ptr<UbxSecSigFrameComms> sig()
  {
    return sig_;
  }

  std::shared_ptr<UbxSecSigLogFrameComms> siglog()
  {
    return siglog_;
  }

  std::shared_ptr<UbxSecUniqidFrameComms> uniqid()
  {
    return uniqid_;
  }
};
}  // namespace ubx::sec

#endif  // UBLOX_DGNSS_NODE__UBX__UBX_SEC_HPP_
