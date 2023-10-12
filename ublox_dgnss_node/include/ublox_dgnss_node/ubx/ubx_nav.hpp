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

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_NAV_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_NAV_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_clock.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_cov.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_dop.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_eoe.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_posecef.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_posllh.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_hpposecef.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_hpposllh.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_odo.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_orb.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_sat.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_sig.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_resetodo.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_pvt.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_status.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_relposned.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_timeutc.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_velecef.hpp"
#include "ublox_dgnss_node/ubx/nav/ubx_nav_velned.hpp"

namespace ubx::nav
{

typedef UBXFrameComms<nav::clock::NavClockPayload, usb::Connection> UbxNavClockFrameComms;
typedef UBXFrameComms<nav::cov::NavCovPayload, usb::Connection> UbxNavCovFrameComms;
typedef UBXFrameComms<nav::dop::NavDOPPayload, usb::Connection> UbxNavDOPFrameComms;
typedef UBXFrameComms<nav::eoe::NavEOEPayload, usb::Connection> UbxNavEOEFrameComms;
typedef UBXFrameComms<nav::posecef::NavPosECEFPayload, usb::Connection> UbxNavPosECEFFrameComms;
typedef UBXFrameComms<nav::hpposecef::NavHPPosECEFPayload,
    usb::Connection> UbxNavHPPosECEFFrameComms;
typedef UBXFrameComms<nav::posllh::NavPosLLHPayload, usb::Connection> UbxNavPosLLHFrameComms;
typedef UBXFrameComms<nav::hpposllh::NavHPPosLLHPayload, usb::Connection> UbxNavHPPosLLHFrameComms;
typedef UBXFrameComms<nav::odo::NavOdoPayload, usb::Connection> UbxNavOdoFrameComms;
typedef UBXFrameComms<nav::orb::NavOrbPayload, usb::Connection> UbxNavOrbFrameComms;
typedef UBXFrameComms<nav::sat::NavSatPayload, usb::Connection> UbxNavSatFrameComms;
typedef UBXFrameComms<nav::sig::NavSigPayload, usb::Connection> UbxNavSigFrameComms;
typedef UBXFrameComms<nav::resetodo::NavResetOdoPayload, usb::Connection> UbxNavResetOdoFrameComms;
typedef UBXFrameComms<nav::pvt::NavPvtPayload, usb::Connection> UbxNavPvtFrameComms;
typedef UBXFrameComms<nav::status::NavStatusPayload, usb::Connection> UbxNavStatusFrameComms;
typedef UBXFrameComms<nav::relposned::NavRelPosNedPayload,
    usb::Connection> UbxNavRelPosNedFrameComms;
typedef UBXFrameComms<nav::timeutc::NavTimeUTCPayload, usb::Connection> UbxNavTimeUTCFrameComms;
typedef UBXFrameComms<nav::velecef::NavVelECEFPayload, usb::Connection> UbxNavVelECEFFrameComms;
typedef UBXFrameComms<nav::velned::NavVelNEDPayload, usb::Connection> UbxNavVelNEDFrameComms;

class UbxNav
{
private:
  std::shared_ptr<usb::Connection> usbc_;

  std::shared_ptr<UbxNavClockFrameComms> clock_;
  std::shared_ptr<UbxNavCovFrameComms> cov_;
  std::shared_ptr<UbxNavDOPFrameComms> dop_;
  std::shared_ptr<UbxNavEOEFrameComms> eoe_;
  std::shared_ptr<UbxNavPosECEFFrameComms> posecef_;
  std::shared_ptr<UbxNavHPPosECEFFrameComms> hpposecef_;
  std::shared_ptr<UbxNavPosLLHFrameComms> posllh_;
  std::shared_ptr<UbxNavHPPosLLHFrameComms> hpposllh_;
  std::shared_ptr<UbxNavOdoFrameComms> odo_;
  std::shared_ptr<UbxNavOrbFrameComms> orb_;
  std::shared_ptr<UbxNavSatFrameComms> sat_;
  std::shared_ptr<UbxNavSigFrameComms> sig_;
  std::shared_ptr<UbxNavResetOdoFrameComms> resetodo_;
  std::shared_ptr<UbxNavPvtFrameComms> pvt_;
  std::shared_ptr<UbxNavStatusFrameComms> status_;
  std::shared_ptr<UbxNavRelPosNedFrameComms> relposned_;
  std::shared_ptr<UbxNavTimeUTCFrameComms> timeutc_;
  std::shared_ptr<UbxNavVelECEFFrameComms> velecef_;
  std::shared_ptr<UbxNavVelNEDFrameComms> velned_;

public:
  explicit UbxNav(std::shared_ptr<usb::Connection> usbc)
  {
    usbc_ = usbc;
    clock_ = std::make_shared<UbxNavClockFrameComms>(usbc_);
    cov_ = std::make_shared<UbxNavCovFrameComms>(usbc_);
    dop_ = std::make_shared<UbxNavDOPFrameComms>(usbc_);
    eoe_ = std::make_shared<UbxNavEOEFrameComms>(usbc_);
    posecef_ = std::make_shared<UbxNavPosECEFFrameComms>(usbc_);
    hpposecef_ = std::make_shared<UbxNavHPPosECEFFrameComms>(usbc_);
    posllh_ = std::make_shared<UbxNavPosLLHFrameComms>(usbc_);
    hpposllh_ = std::make_shared<UbxNavHPPosLLHFrameComms>(usbc_);
    odo_ = std::make_shared<UbxNavOdoFrameComms>(usbc_);
    orb_ = std::make_shared<UbxNavOrbFrameComms>(usbc_);
    sat_ = std::make_shared<UbxNavSatFrameComms>(usbc_);
    sig_ = std::make_shared<UbxNavSigFrameComms>(usbc_);
    resetodo_ = std::make_shared<UbxNavResetOdoFrameComms>(usbc_);
    pvt_ = std::make_shared<UbxNavPvtFrameComms>(usbc_);
    status_ = std::make_shared<UbxNavStatusFrameComms>(usbc_);
    relposned_ = std::make_shared<UbxNavRelPosNedFrameComms>(usbc_);
    timeutc_ = std::make_shared<UbxNavTimeUTCFrameComms>(usbc_);
    velecef_ = std::make_shared<UbxNavVelECEFFrameComms>(usbc_);
    velned_ = std::make_shared<UbxNavVelNEDFrameComms>(usbc_);
  }

  std::shared_ptr<UbxNavClockFrameComms> clock()
  {
    return clock_;
  }
  std::shared_ptr<UbxNavCovFrameComms> cov()
  {
    return cov_;
  }
  std::shared_ptr<UbxNavDOPFrameComms> dop()
  {
    return dop_;
  }
  std::shared_ptr<UbxNavEOEFrameComms> eoe()
  {
    return eoe_;
  }
  std::shared_ptr<UbxNavPosECEFFrameComms> posecef()
  {
    return posecef_;
  }
  std::shared_ptr<UbxNavHPPosECEFFrameComms> hpposecef()
  {
    return hpposecef_;
  }
  std::shared_ptr<UbxNavPosLLHFrameComms> posllh()
  {
    return posllh_;
  }
  std::shared_ptr<UbxNavHPPosLLHFrameComms> hpposllh()
  {
    return hpposllh_;
  }
  std::shared_ptr<UbxNavOdoFrameComms> odo()
  {
    return odo_;
  }
  std::shared_ptr<UbxNavOrbFrameComms> orb()
  {
    return orb_;
  }
  std::shared_ptr<UbxNavSatFrameComms> sat()
  {
    return sat_;
  }
  std::shared_ptr<UbxNavSigFrameComms> sig()
  {
    return sig_;
  }
  std::shared_ptr<UbxNavResetOdoFrameComms> resetodo()
  {
    return resetodo_;
  }
  std::shared_ptr<UbxNavPvtFrameComms> pvt()
  {
    return pvt_;
  }
  std::shared_ptr<UbxNavStatusFrameComms> status()
  {
    return status_;
  }
  std::shared_ptr<UbxNavRelPosNedFrameComms> relposned()
  {
    return relposned_;
  }
  std::shared_ptr<UbxNavTimeUTCFrameComms> timeutc()
  {
    return timeutc_;
  }
  std::shared_ptr<UbxNavVelECEFFrameComms> velecef()
  {
    return velecef_;
  }
  std::shared_ptr<UbxNavVelNEDFrameComms> velned()
  {
    return velned_;
  }
  void frame(std::shared_ptr<ubx::Frame> frame)
  {
    switch (frame->msg_id) {
      case ubx::UBX_NAV_CLOCK:
        clock_->frame(frame);
        break;
      case ubx::UBX_NAV_COV:
        cov_->frame(frame);
        break;
      case ubx::UBX_NAV_DOP:
        dop_->frame(frame);
        break;
      case ubx::UBX_NAV_EOE:
        eoe_->frame(frame);
        break;
      case ubx::UBX_NAV_POSLLH:
        posllh_->frame(frame);
        break;
      case ubx::UBX_NAV_HPPOSLLH:
        hpposllh_->frame(frame);
        break;
      case ubx::UBX_NAV_POSECEF:
        posecef_->frame(frame);
        break;
      case ubx::UBX_NAV_HPPOSECEF:
        hpposecef_->frame(frame);
        break;
      case ubx::UBX_NAV_ODO:
        odo_->frame(frame);
        break;
      case ubx::UBX_NAV_ORB:
        orb_->frame(frame);
        break;
      case ubx::UBX_NAV_SAT:
        sat_->frame(frame);
        break;
      case ubx::UBX_NAV_SIG:
        sig_->frame(frame);
        break;
      case ubx::UBX_NAV_RESETODO:
        resetodo_->frame(frame);
        break;
      case ubx::UBX_NAV_PVT:
        pvt_->frame(frame);
        break;
      case ubx::UBX_NAV_STATUS:
        status_->frame(frame);
        break;
      case ubx::UBX_NAV_RELPOSNED:
        relposned_->frame(frame);
        break;
      case ubx::UBX_NAV_TIMEUTC:
        timeutc_->frame(frame);
        break;
      case ubx::UBX_NAV_VELECEF:
        velecef_->frame(frame);
        break;
      case ubx::UBX_NAV_VELNED:
        velned_->frame(frame);
        break;
      default:
        // break;
        throw UbxValueException("unknown UBX_NAV msg_id: " + ubx::to_hex(frame->msg_id));
    }
  }
};
}  // namespace ubx::nav

#endif  // UBLOX_DGNSS_NODE__UBX__UBX_NAV_HPP_
