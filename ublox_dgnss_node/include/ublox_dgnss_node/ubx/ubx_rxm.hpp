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
#include "ublox_dgnss_node/ubx/rxm/ubx_rxm_measx.hpp"
#include "ublox_dgnss_node/ubx/rxm/ubx_rxm_rawx.hpp"
#include "ublox_dgnss_node/ubx/rxm/ubx_rxm_spartn.hpp"
#include "ublox_dgnss_node/ubx/rxm/ubx_rxm_spartn_key.hpp"
#include "ublox_dgnss_node/ubx/rxm/ubx_rxm_sfrbx.hpp"
#include "ublox_dgnss_node/ubx/rxm/ubx_rxm_cor.hpp"
#include "ublox_dgnss_node/ubx/rxm/ubx_rxm_pmp.hpp"
#include "ublox_dgnss_node/ubx/rxm/ubx_rxm_qzssl6.hpp"

namespace ubx::rxm
{

typedef UBXFrameComms<rxm::rtcm::RxmRTCMPayload, usb::Connection> UbxRxmRTCMFrameComms;
typedef UBXFrameComms<rxm::measx::RxmMeasxPayload, usb::Connection> UbxRxmMeasxFrameComms;
typedef UBXFrameComms<rxm::rawx::RxmRawxPayload, usb::Connection> UbxRxmRawxFrameComms;
typedef UBXFrameComms<rxm::spartn::RxmSpartnPayload, usb::Connection> UbxRxmSpartnFrameComms;
typedef UBXFrameComms<rxm::sfrbx::RxmSfrbxPayload, usb::Connection> UbxRxmSfrbxFrameComms;
typedef UBXFrameComms<rxm::cor::RxmCorPayload, usb::Connection> UbxRxmCorFrameComms;
typedef UBXFrameComms<rxm::pmp::RxmPmpPayload, usb::Connection> UbxRxmPmpFrameComms;
typedef UBXFrameComms<rxm::qzssl6::RxmQzssl6Payload, usb::Connection> UbxRxmQzssl6FrameComms;
typedef UBXFrameComms<rxm::spartnkey::RxmSpartnKeyFullPayload,
    usb::Connection> UbxRxmSpartnKeyFullFrameComms;

class UbxRxm
{
private:
  std::shared_ptr<usb::Connection> usbc_;
  std::shared_ptr<UbxRxmRTCMFrameComms> rtcm_;
  std::shared_ptr<UbxRxmMeasxFrameComms> measx_;
  std::shared_ptr<UbxRxmRawxFrameComms> rawx_;
  std::shared_ptr<UbxRxmSpartnFrameComms> spartn_;
  std::shared_ptr<UbxRxmSfrbxFrameComms> sfrbx_;
  std::shared_ptr<UbxRxmCorFrameComms> cor_;
  std::shared_ptr<UbxRxmPmpFrameComms> pmp_;
  std::shared_ptr<UbxRxmQzssl6FrameComms> qzssl6_;
  std::shared_ptr<UbxRxmSpartnKeyFullFrameComms> spartn_key_full_;

public:
  explicit UbxRxm(std::shared_ptr<usb::Connection> usbc)
  {
    usbc_ = usbc;
    rtcm_ = std::make_shared<UbxRxmRTCMFrameComms>(usbc_);
    measx_ = std::make_shared<UbxRxmMeasxFrameComms>(usbc_);
    rawx_ = std::make_shared<UbxRxmRawxFrameComms>(usbc_);
    spartn_ = std::make_shared<UbxRxmSpartnFrameComms>(usbc_);
    sfrbx_ = std::make_shared<UbxRxmSfrbxFrameComms>(usbc_);
    cor_ = std::make_shared<UbxRxmCorFrameComms>(usbc_);
    pmp_ = std::make_shared<UbxRxmPmpFrameComms>(usbc_);
    qzssl6_ = std::make_shared<UbxRxmQzssl6FrameComms>(usbc_);
    spartn_key_full_ = std::make_shared<UbxRxmSpartnKeyFullFrameComms>(usbc_);
  }
  std::shared_ptr<UbxRxmRTCMFrameComms> rtcm()
  {
    return rtcm_;
  }
  std::shared_ptr<UbxRxmMeasxFrameComms> measx()
  {
    return measx_;
  }
  std::shared_ptr<UbxRxmRawxFrameComms> rawx()
  {
    return rawx_;
  }
  std::shared_ptr<UbxRxmSpartnFrameComms> spartn()
  {
    return spartn_;
  }
  std::shared_ptr<UbxRxmSfrbxFrameComms> sfrbx()
  {
    return sfrbx_;
  }
  std::shared_ptr<UbxRxmCorFrameComms> cor()
  {
    return cor_;
  }
  std::shared_ptr<UbxRxmPmpFrameComms> pmp()
  {
    return pmp_;
  }
  std::shared_ptr<UbxRxmQzssl6FrameComms> qzssl6()
  {
    return qzssl6_;
  }
  std::shared_ptr<UbxRxmSpartnKeyFullFrameComms> spartnkey_full()
  {
    return spartn_key_full_;
  }
};
}  // namespace ubx::rxm
#endif  // UBLOX_DGNSS_NODE__UBX__UBX_RXM_HPP_
