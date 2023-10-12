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

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_MSG_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_MSG_HPP_

#include "ublox_dgnss_node/ubx/ubx_types.hpp"
namespace ubx
{
const msg_class_t UBX_ACK = 0x05;
const msg_id_t UBX_ACK_NAK = 0x00;
const msg_id_t UBX_ACK_ACK = 0x01;

const msg_class_t UBX_INF = 0x04;
const msg_id_t UBX_INF_DEBUG = 0x04;
const msg_id_t UBX_INF_ERROR = 0x00;
const msg_id_t UBX_INF_NOTICE = 0x02;
const msg_id_t UBX_INF_TEST = 0x03;
const msg_id_t UBX_INF_WARNING = 0x01;

const msg_class_t UBX_CFG = 0x06;
const msg_id_t UBX_CFG_VALSET = 0x8a;
const msg_id_t UBX_CFG_VALGET = 0x8b;
const msg_id_t UBX_CFG_VALDEL = 0x8c;
const msg_id_t UBX_CFG_RST = 0x04;

const msg_class_t UBX_MON = 0x0A;
const msg_id_t UBX_MON_VER = 0x04;

const msg_class_t UBX_NAV = 0x01;
const msg_id_t UBX_NAV_CLOCK = 0x22;
const msg_id_t UBX_NAV_COV = 0x36;
const msg_id_t UBX_NAV_DOP = 0x04;
const msg_id_t UBX_NAV_EOE = 0x61;
const msg_id_t UBX_NAV_POSECEF = 0x01;
const msg_id_t UBX_NAV_POSLLH = 0x02;
const msg_id_t UBX_NAV_STATUS = 0x03;
const msg_id_t UBX_NAV_PVT = 0x07;
const msg_id_t UBX_NAV_HPPOSECEF = 0x13;
const msg_id_t UBX_NAV_HPPOSLLH = 0x14;
const msg_id_t UBX_NAV_ODO = 0x09;
const msg_id_t UBX_NAV_ORB = 0x34;
const msg_id_t UBX_NAV_SAT = 0x35;
const msg_id_t UBX_NAV_SIG = 0x43;
const msg_id_t UBX_NAV_RESETODO = 0x10;
const msg_id_t UBX_NAV_TIMEUTC = 0x21;
const msg_id_t UBX_NAV_VELECEF = 0x11;
const msg_id_t UBX_NAV_VELNED = 0x12;
const msg_id_t UBX_NAV_RELPOSNED = 0x3c;

const msg_class_t UBX_RXM = 0x02;
const msg_id_t UBX_RXM_RTCM = 0x32;
const msg_id_t UBX_RXM_MEASX = 0x14;
const msg_id_t UBX_RXM_RAWX = 0x15;

const msg_class_t UBX_SEC = 0x27;
const msg_id_t UBX_SEC_SIG = 0x09;
const msg_id_t UBX_SEC_SIGLOG = 0x10;
const msg_id_t UBX_SEC_UNIQID = 0x03;

const msg_class_t UBX_ESF = 0x10;
const msg_id_t UBX_ESF_ALG = 0x14;
const msg_id_t UBX_ESF_INS = 0x15;
const msg_id_t UBX_ESF_MEAS = 0x02;
const msg_id_t UBX_ESF_RAW = 0x03;
const msg_id_t UBX_ESF_STATUS = 0x10;
}  // namespace ubx

#endif  // UBLOX_DGNSS_NODE__UBX__UBX_MSG_HPP_
