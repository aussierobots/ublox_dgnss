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

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_CFG_ITEM_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_CFG_ITEM_HPP_
#include <unistd.h>
#include <map>
#include <list>
#include <string>
#include <sstream>
#include "ublox_dgnss_node/ubx/ubx_types.hpp"
#include "ublox_dgnss_node/ubx/ubx.hpp"

namespace ubx::cfg
{
size_t storage_size_bytes(u8_t storage_size_id)
{
  size_t size = 0;
  switch (storage_size_id) {
    case 0x01: size = 1; break;     /// one bit but uses one byte
    case 0x02: size = 1; break;
    case 0x03: size = 2; break;
    case 0x04: size = 4; break;
    case 0x05: size = 8; break;
    default:
      size = 0;
  }
  return size;
}

union ubx_key_id_t {
  u4_t all;
  struct  ubx_key_id_bit_t
  {
    u2_t item_id : 12;
    u1_t reserved2 : 4;
    u1_t group_id : 8;
    u1_t reserverd1 : 4;
    u1_t storage_size_id : 3;
    u1_t reserved0 : 1;
  } bits;
  u4_t key_id()
  {
    return all;
  }
  u1_t storage_size_id()
  {
    return bits.storage_size_id;
  }
  u1_t group_id()
  {
    return bits.group_id;
  }
  u2_t item_id()
  {
    return bits.item_id;
  }                                         // actually 12 bits
  size_t storage_size()
  {
    return storage_size_bytes(bits.storage_size_id);
  }
  std::string to_hex()
  {
    std::ostringstream oss;
    oss << "0x" << std::setfill('0') << std::setw(8) << std::right << std::hex << +all;
    return oss.str();
  }
};


struct key_value_t
{
  ubx_key_id_t ubx_key_id;
  value_t ubx_value;
};

enum ubx_unit_t
{
  NA,       // not applicable
  M,       // meters
  Y,       // years
  MONTH,       // months
  D,       // days
  H,       // hours
  MIN,       // minutes
  S,       // seconds
  HZ,       // frquency (Hz)
  MA,       // milli amps (mA)
  MS,       // milliseconds
  DEG,       // degree ie Longitude, Latitude
  MM,       // milimeter
  CM,       // centimeter
  MPS,       // meters per second
};

struct ubx_cfg_item_t
{
  const char * ubx_config_item;
  ubx_key_id_t ubx_key_id;
  ubx_type_t ubx_type;
  double_t scale;
  ubx_unit_t ubx_unit;
};

// cfg infmsg
const ubx_cfg_item_t CFG_INFMSG_UBX_USB = {"CFG_INFMSG_UBX_USB", 0x20920004, X1, 1, NA};
const ubx_cfg_item_t CFG_INFMSG_NMEA_USB = {"CFG_INFMSG_NMEA_USB", 0x20920009, X1, 1, NA};
enum CFG_INFMSG_ENUM
{
  INFMSG_ERROR = 0x01,         // Enable ERROR information messages
  INFMSG_WARNING = 0x02,       // Enable WARNING information messages
  INFMSG_NOTICE = 0x04,        // Enable NOTICE information messages
  INFMSG_TEST = 0x08,          // Enable TEST information messages
  INFMSG_DEBUG = 0x10,         // Enable DEBUG information messages
};

// cfg uart1
const ubx_cfg_item_t CFG_UART1INPROT_UBX = {"CFG_UART1INPROT_UBX", 0x10730001, L, 1, NA};
const ubx_cfg_item_t CFG_UART1INPROT_NMEA = {"CFG_UART1INPROT_NMEA", 0x10730002, L, 1, NA};
const ubx_cfg_item_t CFG_UART1INPROT_RTCM3X = {"CFG_UART1INPROT_RTCM3X", 0x10730004, L, 1, NA};
const ubx_cfg_item_t CFG_UART1OUTPROT_UBX = {"CFG_UART1OUTPROT_UBX", 0x10740001, L, 1, NA};
const ubx_cfg_item_t CFG_UART1OUTPROT_NMEA = {"CFG_UART1OUTPROT_NMEA", 0x10740002, L, 1, NA};
const ubx_cfg_item_t CFG_UART1OUTPROT_RTCM3X = {"CFG_UART1OUTPROT_RTCM3X", 0x10740004, L, 1, NA};

// cfg usb
const ubx_cfg_item_t CFG_USBINPROT_UBX = {"CFG_USBINPROT_UBX", 0x10770001, L, 1, NA};
const ubx_cfg_item_t CFG_USBINPROT_NMEA = {"CFG_USBINPROT_NMEA", 0x10770002, L, 1, NA};
const ubx_cfg_item_t CFG_USBINPROT_RTCM3X = {"CFG_USBINPROT_RTCM3X", 0x10770004, L, 1, NA};
const ubx_cfg_item_t CFG_USBOUTPROT_UBX = {"CFG_USBOUTPROT_UBX", 0x10780001, L, 1, NA};
const ubx_cfg_item_t CFG_USBOUTPROT_NMEA = {"CFG_USBOUTPROT_NMEA", 0x10780002, L, 1, NA};
const ubx_cfg_item_t CFG_USBOUTPROT_RTCM3X = {"CFG_USBOUTPROT_RTCM3X", 0x10780004, L, 1, NA};

// cfg tmode - time mode configuration
const ubx_cfg_item_t CFG_TMODE_MODE = {"CFG_TMODE_MODE", 0x20030001, E1, 1, NA};
enum CFG_TMODE_MODE_ENUM {DISABLED = 0, SURVEY_IN = 1, FIXED = 2};
const ubx_cfg_item_t CFG_TMODE_POS_TYPE = {"CFG_TMODE_POS_TYPE", 0x20030002, E1, 1, NA};
enum CFG_TMODE_POS_TYPE_ENUM {ECEF = 0, LLH = 1};

// cfg navhpg - high precision navigation configuration
const ubx_cfg_item_t CFG_NAVHPG_DGNSSMODE = {"CFG_NAVHPG_DGNSSMODE", 0x20140011, E1, 1, NA};
enum CFG_NAVHPG_DGNSSMODE_ENUM
{
  RTK_FLOAT = 2,       // no attempts made to fix ambiguities
  RTK_FIXED = 3       // Ambiguities are fixed whenever possible
};

// cfg navspg - standard precision navigation configuration
const ubx_cfg_item_t CFG_NAVSPG_FIXMODE = {"CFG_NAVSPG_FIXMODE", 0x20110011, E1, 1, NA};
enum CFG_NAVSPG_FIXMODE_ENUM
{
  ONLY_2D = 1,         // 2D Only
  ONLY_3D = 2,         // 3D only
  AUTO = 3             // Auto 2D/3D
};
const ubx_cfg_item_t CFG_NAVSPG_INIFIX3D = {"CFG_NAVSPG_INIFIX3D", 0x10110013, L, 1, NA};

const ubx_cfg_item_t CFG_NAVSPG_UTCSTANDARD = {"CFG_NAVSPG_UTCSTANDARD", 0x2011001c, E1, 1, NA};
enum CFG_NAVSPG_UTCSTANDARD_ENUM
{
  UTC_AUTO = 0,         // Automatic; receiver selects based on GNSS configuration
  UTC_USNO = 3,         // UTC as operated by the US Naval Observatory (USNO)
                        //  derived from GPS time
  UTC_EU = 5,           // UT as combined from multiple European laboratories
                        //  dervied from Galileo time
  UTC_SU = 6,           // UTC as operated by the former Soviet Unnion (SU)
                        //  derived from GLONASS time
  UTC_NTSC = 7          // UTC as operated by the National Time Service Center (NTSC), Chin
                        //  derived from BeiDou time
};

const ubx_cfg_item_t CFG_NAVSPG_DYNMODEL = {"CFG_NAVSPG_DYNMODEL", 0x20110021, E1, 1, NA};
enum CFG_NAVSPG_DYNMODEL_ENUM
{
  DYN_MODEL_PORT = 0,       // Portable
  DYN_MODEL_STAT = 2,       // Stationary
  DYN_MODEL_PED = 3,       // Pedestrian
  DYN_MODEL_AUTOMOT = 4,       // Automotive
  DYN_MODEL_SEA = 5,          // Sea
  DYN_MODEL_AIR1 = 6,         // Airborne <1g acceleration
  DYN_MODEL_AIR2 = 7,         // Airborne <2g acceleration
  DYN_MODEL_AIR4 = 8,         // Airborne <4g acceleration
  DYN_MODEL_WRIST = 9,        // Wrist-worn watch (not available in all products)
  DYN_MODEL_BIKE = 10,        // Motorbike (not available in all products)
  DYN_MODEL_MOWER = 11,       // Robotic lawn mower (not available in all products)
  DYN_MODEL_ESCOOTER = 12,    // E-scooter (not available in all products)
};

// cfg odo
const ubx_cfg_item_t CFG_ODO_USE_ODO = {"CFG_ODO_USE_ODO", 0x10220001, L, 1, NA};
const ubx_cfg_item_t CFG_ODO_USE_COG = {"CFG_ODO_USE_COG", 0x10220002, L, 1, NA};
const ubx_cfg_item_t CFG_ODO_OUTLPVEL = {"CFG_ODO_OUTLPVEL", 0x10220003, L, 1, NA};
const ubx_cfg_item_t CFG_ODO_OUTLPCOG = {"CFG_ODO_OUTLPCOG", 0x10220004, L, 1, NA};
const ubx_cfg_item_t CFG_ODO_PROFILE = {"CFG_ODO_PROFILE", 0x20220005, E1, 1, NA};
enum CFG_ODO_PROFILE_ENUM
{
  ODO_RUN = 0,          // Running
  ODO_CYCL = 1,         // Cycling
  ODO_SWIM = 2,         // Swimming
  ODO_CAR = 3,          // Car
  ODO_CUSTOM = 4,       // Custom
};
const ubx_cfg_item_t CFG_ODO_COGMAXSPEED = {"CFG_ODO_COGMAXSPEED", 0x20220021, U1, 1, MPS};
const ubx_cfg_item_t CFG_ODO_COGMAXPOSACC = {"CFG_ODO_COGMAXPOSACC", 0x20220022, U1, 1, NA};
const ubx_cfg_item_t CFG_ODO_VELLPGAIN = {"CFG_ODO_VALLPGAIN", 0x20220031, U1, 1, NA};
const ubx_cfg_item_t CFG_ODO_COGLPGAIN = {"CFG_ODO_COGLPGAIN", 0x20220032, U1, 1, NA};


// cfg rate - navigation and measurement rate configuration
const ubx_cfg_item_t CFG_RATE_MEAS = {"CFG_RATE_MEAS", 0x30210001, U2, .001, S};
const ubx_cfg_item_t CFG_RATE_NAV = {"CFG_RATE_NAV", 0x30210002, U2, 1, NA};
const ubx_cfg_item_t CFG_RATE_TIMEREF = {"CFG_RATE_TIMEREF", 0x30210003, E1, 1, NA};
enum CFG_RATE_TIMEREF_ENUM
{
  ALIGN_UTC = 0,         // Align measurements to UTC time
  ALIGN_GPS = 1,         // Align measurements to GPS time
  ALIGN_GLO = 2,         // Align measurements to GLONASS time
  ALIGN_BDS = 3,         // Align measurements to BeiDou time
  ALIGN_GAL = 4,         // Align measurements to Galileo time
};

// cfg sfimu - sensor fusion IMU configuration
const ubx_cfg_item_t CFG_SFIMU_AUTO_MNTALG_ENA =
{"CFG_SFIMU_AUTO_MNTALG_ENA", 0x10060027, L, 0, NA};
const ubx_cfg_item_t CFG_SFIMU_IMU_MNTALG_YAW =
{"CFG_SFIMU_IMU_MNTALG_YAW", 0x4006002d, U4, .001, DEG};
const ubx_cfg_item_t CFG_SFIMU_IMU_MNTALG_PITCH =
{"CFG_SFIMU_IMU_MNTALG_PITCH", 0x3006002e, I2, .001, DEG};
const ubx_cfg_item_t CFG_SFIMU_IMU_MNTALG_ROLL =
{"CFG_SFIMU_IMU_MNTALG_ROLL", 0x3006002f, I2, .001, DEG};

// cfg sfodo - sensor fusion odometer configuration
const ubx_cfg_item_t CFG_SFODO_COMBINE_TICKS =
{"CFG_SFODO_COMBINE_TICKS", 0x10070001, L, 0, NA};
const ubx_cfg_item_t CFG_SFODO_COUNT_MAX =
{"CFG_SFODO_COUNT_MAX", 0x40070009, U4, 0, NA};
const ubx_cfg_item_t CFG_SFODO_DIS_AUTOCOUNTMAX =
{"CFG_SFODO_DIS_AUTOCOUNTMAX", 0x10070004, L, 0, NA};
const ubx_cfg_item_t CFG_SFODO_DIS_AUTODIRPINPOL =
{"CFG_SFODO_DIS_AUTODIRPINPOL", 0x10070005, L, 0, NA};
const ubx_cfg_item_t CFG_SFODO_FACTOR =
{"CFG_SFODO_FACTOR", 0x40070007, U4, 1e-6, NA};
const ubx_cfg_item_t CFG_SFODO_QUANT_ERROR =
{"CFG_SFODO_QUANT_ERROR", 0x40070008, U4, 1e-6, M};
const ubx_cfg_item_t CFG_SFODO_LATENCY =
{"CFG_SFODO_LATENCY", 0x3007000a, U2, 0, MS};

// cfg msgout - msg output rate configurations
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_STATUS_USB =
{"CFG_MSGOUT_UBX_NAV_STATUS_USB", 0x2091001d, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_CLOCK_USB =
{"CFG_MSGOUT_UBX_NAV_CLOCK_USB", 0x20910068, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_COV_USB =
{"CFG_MSGOUT_UBX_NAV_COV_USB", 0x20910086, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_DOP_USB =
{"CFG_MSGOUT_UBX_NAV_DOP_USB", 0x2091003b, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_EOE_USB =
{"CFG_MSGOUT_UBX_NAV_EOE_USB", 0x20910162, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_POSECEF_USB =
{"CFG_MSGOUT_UBX_NAV_POSECEF_USB", 0x20910027, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_HPPOSECEF_USB =
{"CFG_MSGOUT_UBX_NAV_HPPOSECEF_USB", 0x20910031, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_POSLLH_USB =
{"CFG_MSGOUT_UBX_NAV_POSLLH_USB", 0x2091002C, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB =
{"CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB", 0x20910036, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_ODO_USB =
{"CFG_MSGOUT_UBX_NAV_ODO_USB", 0x20910081, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_ORB_USB =
{"CFG_MSGOUT_UBX_NAV_ORB_USB", 0x20910013, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_SAT_USB =
{"CFG_MSGOUT_UBX_NAV_SAT_USB", 0x20910018, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_SIG_USB =
{"CFG_MSGOUT_UBX_NAV_SIG_USB", 0x20910348, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_PVT_USB =
{"CFG_MSGOUT_UBX_NAV_PVT_USB", 0x20910009, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_RELPOSNED_USB =
{"CFG_MSGOUT_UBX_NAV_RELPOSNED_USB", 0x20910090, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_TIMEUTC_USB =
{"CFG_MSGOUT_UBX_NAV_TIMEUTC_USB", 0x2091005e, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_VELECEF_USB =
{"CFG_MSGOUT_UBX_NAV_VELECEF_USB", 0x20910040, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_NAV_VELNED_USB =
{"CFG_MSGOUT_UBX_NAV_VELNED_USB", 0x20910045, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_RXM_RTCM_USB =
{"CFG_MSGOUT_UBX_RXM_RTCM_USB", 0x2091026b, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_RXM_MEASX_USB =
{"CFG_MSGOUT_UBX_RXM_MEASX_USB", 0x20910207, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_RXM_RAWX_USB =
{"CFG_MSGOUT_UBX_RXM_RAWX_USB", 0x209102a7, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_ESF_MEAS_USB =
{"CFG_MSGOUT_UBX_ESF_MEAS_USB", 0x2091027a, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_ESF_STATUS_USB =
{"CFG_MSGOUT_UBX_ESF_STATUS_USB", 0x20910108, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_SEC_SIG_USB =
{"CFG_MSGOUT_UBX_SEC_SIG_USB", 0x20910637, U1, 0, NA};
const ubx_cfg_item_t CFG_MSGOUT_UBX_SEC_SIGLOG_USB =
{"CFG_MSGOUT_UBX_SEC_SIGLOG_USB", 0x2091068c, U1, 0, NA};

// cfg itfm - Jamming and interference monitor configuration
const ubx_cfg_item_t CFG_ITFM_BBTHRESHOLD = {"CFG_ITFM_BBTHRESHOLD", 0x20410001, U1, 1, NA};
const ubx_cfg_item_t CFG_ITFM_CWTHRESHOLD = {"CFG_ITFM_CWTHRESHOLD", 0x20410002, U1, 1, NA};
const ubx_cfg_item_t CFG_ITFM_ENABLE = {"CFG_ITFM_ENABLE", 0x1041000d, L, 0, NA};
const ubx_cfg_item_t CFG_ITFM_ANTSETTING = {"CFG_ITFM_ANTSETTING", 0x20410010, E1, 0, NA};
const ubx_cfg_item_t CFG_ITFM_ENABLE_AUX = {"CFG_ITFM_ENABLE_AUX", 0x10410013, L, 0, NA};

enum CFG_ITFM_ANTSETTING_ENUM
{
  UNKNOWN = 0,
  PASSIVE = 1,
  ACTIVE = 2,
};

std::map<ubx_key_id_t, ubx_cfg_item_t> ubxKeyCfgItemMap = {
  {CFG_INFMSG_UBX_USB.ubx_key_id, CFG_INFMSG_UBX_USB},
  {CFG_INFMSG_NMEA_USB.ubx_key_id, CFG_INFMSG_NMEA_USB},
  {CFG_UART1INPROT_UBX.ubx_key_id, CFG_UART1INPROT_UBX},
  {CFG_UART1INPROT_NMEA.ubx_key_id, CFG_UART1INPROT_NMEA},
  {CFG_UART1INPROT_RTCM3X.ubx_key_id, CFG_UART1INPROT_RTCM3X},
  {CFG_UART1OUTPROT_UBX.ubx_key_id, CFG_UART1OUTPROT_UBX},
  {CFG_UART1OUTPROT_NMEA.ubx_key_id, CFG_UART1OUTPROT_NMEA},
  {CFG_UART1OUTPROT_RTCM3X.ubx_key_id, CFG_UART1OUTPROT_RTCM3X},
  {CFG_USBINPROT_UBX.ubx_key_id, CFG_USBINPROT_UBX},
  {CFG_USBINPROT_NMEA.ubx_key_id, CFG_USBINPROT_NMEA},
  {CFG_USBINPROT_RTCM3X.ubx_key_id, CFG_USBINPROT_RTCM3X},
  {CFG_USBOUTPROT_UBX.ubx_key_id, CFG_USBOUTPROT_UBX},
  {CFG_USBOUTPROT_NMEA.ubx_key_id, CFG_USBOUTPROT_NMEA},
  {CFG_USBOUTPROT_RTCM3X.ubx_key_id, CFG_USBOUTPROT_RTCM3X},
  {CFG_TMODE_MODE.ubx_key_id, CFG_TMODE_MODE},
  {CFG_TMODE_POS_TYPE.ubx_key_id, CFG_TMODE_POS_TYPE},
  {CFG_NAVHPG_DGNSSMODE.ubx_key_id, CFG_NAVHPG_DGNSSMODE},
  {CFG_NAVSPG_FIXMODE.ubx_key_id, CFG_NAVSPG_FIXMODE},
  {CFG_NAVSPG_INIFIX3D.ubx_key_id, CFG_NAVSPG_INIFIX3D},
  {CFG_NAVSPG_UTCSTANDARD.ubx_key_id, CFG_NAVSPG_UTCSTANDARD},
  {CFG_NAVSPG_DYNMODEL.ubx_key_id, CFG_NAVSPG_DYNMODEL},

  {CFG_ODO_USE_ODO.ubx_key_id, CFG_ODO_USE_ODO},
  {CFG_ODO_USE_COG.ubx_key_id, CFG_ODO_USE_COG},
  {CFG_ODO_OUTLPVEL.ubx_key_id, CFG_ODO_OUTLPVEL},
  {CFG_ODO_OUTLPCOG.ubx_key_id, CFG_ODO_OUTLPCOG},
  {CFG_ODO_PROFILE.ubx_key_id, CFG_ODO_PROFILE},
  {CFG_ODO_COGMAXSPEED.ubx_key_id, CFG_ODO_COGMAXSPEED},
  {CFG_ODO_COGMAXPOSACC.ubx_key_id, CFG_ODO_COGMAXPOSACC},
  {CFG_ODO_VELLPGAIN.ubx_key_id, CFG_ODO_VELLPGAIN},
  {CFG_ODO_COGLPGAIN.ubx_key_id, CFG_ODO_COGLPGAIN},

  {CFG_RATE_MEAS.ubx_key_id, CFG_RATE_MEAS},
  {CFG_RATE_NAV.ubx_key_id, CFG_RATE_NAV},
  {CFG_RATE_TIMEREF.ubx_key_id, CFG_RATE_TIMEREF},

  {CFG_SFIMU_AUTO_MNTALG_ENA.ubx_key_id, CFG_SFIMU_AUTO_MNTALG_ENA},
  {CFG_SFIMU_IMU_MNTALG_YAW.ubx_key_id, CFG_SFIMU_IMU_MNTALG_YAW},
  {CFG_SFIMU_IMU_MNTALG_PITCH.ubx_key_id, CFG_SFIMU_IMU_MNTALG_PITCH},
  {CFG_SFIMU_IMU_MNTALG_ROLL.ubx_key_id, CFG_SFIMU_IMU_MNTALG_ROLL},

  {CFG_SFODO_COMBINE_TICKS.ubx_key_id, CFG_SFODO_COMBINE_TICKS},
  {CFG_SFODO_COUNT_MAX.ubx_key_id, CFG_SFODO_COUNT_MAX},
  {CFG_SFODO_DIS_AUTOCOUNTMAX.ubx_key_id, CFG_SFODO_DIS_AUTOCOUNTMAX},
  {CFG_SFODO_DIS_AUTODIRPINPOL.ubx_key_id, CFG_SFODO_DIS_AUTODIRPINPOL},
  {CFG_SFODO_FACTOR.ubx_key_id, CFG_SFODO_FACTOR},
  {CFG_SFODO_LATENCY.ubx_key_id, CFG_SFODO_LATENCY},
  {CFG_SFODO_QUANT_ERROR.ubx_key_id, CFG_SFODO_QUANT_ERROR},

  {CFG_MSGOUT_UBX_NAV_STATUS_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_STATUS_USB},
  {CFG_MSGOUT_UBX_NAV_CLOCK_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_CLOCK_USB},
  {CFG_MSGOUT_UBX_NAV_COV_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_COV_USB},
  {CFG_MSGOUT_UBX_NAV_DOP_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_DOP_USB},
  {CFG_MSGOUT_UBX_NAV_EOE_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_EOE_USB},
  {CFG_MSGOUT_UBX_NAV_POSECEF_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_POSECEF_USB},
  {CFG_MSGOUT_UBX_NAV_HPPOSECEF_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_HPPOSECEF_USB},
  {CFG_MSGOUT_UBX_NAV_POSLLH_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_POSLLH_USB},
  {CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB},
  {CFG_MSGOUT_UBX_NAV_ODO_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_ODO_USB},
  {CFG_MSGOUT_UBX_NAV_ORB_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_ORB_USB},
  {CFG_MSGOUT_UBX_NAV_SAT_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_SAT_USB},
  {CFG_MSGOUT_UBX_NAV_SIG_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_SIG_USB},
  {CFG_MSGOUT_UBX_NAV_PVT_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_PVT_USB},
  {CFG_MSGOUT_UBX_NAV_RELPOSNED_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_RELPOSNED_USB},
  {CFG_MSGOUT_UBX_NAV_TIMEUTC_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_TIMEUTC_USB},
  {CFG_MSGOUT_UBX_NAV_VELECEF_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_VELECEF_USB},
  {CFG_MSGOUT_UBX_NAV_VELNED_USB.ubx_key_id, CFG_MSGOUT_UBX_NAV_VELNED_USB},
  {CFG_MSGOUT_UBX_RXM_RTCM_USB.ubx_key_id, CFG_MSGOUT_UBX_RXM_RTCM_USB},
  {CFG_MSGOUT_UBX_RXM_MEASX_USB.ubx_key_id, CFG_MSGOUT_UBX_RXM_MEASX_USB},
  {CFG_MSGOUT_UBX_RXM_RAWX_USB.ubx_key_id, CFG_MSGOUT_UBX_RXM_RAWX_USB},
  {CFG_MSGOUT_UBX_ESF_MEAS_USB.ubx_key_id, CFG_MSGOUT_UBX_ESF_MEAS_USB},
  {CFG_MSGOUT_UBX_ESF_STATUS_USB.ubx_key_id, CFG_MSGOUT_UBX_ESF_STATUS_USB},
  {CFG_MSGOUT_UBX_SEC_SIG_USB.ubx_key_id, CFG_MSGOUT_UBX_SEC_SIG_USB},
  {CFG_MSGOUT_UBX_SEC_SIGLOG_USB.ubx_key_id, CFG_MSGOUT_UBX_SEC_SIGLOG_USB},

  {CFG_ITFM_BBTHRESHOLD.ubx_key_id, CFG_ITFM_BBTHRESHOLD},
  {CFG_ITFM_CWTHRESHOLD.ubx_key_id, CFG_ITFM_CWTHRESHOLD},
  {CFG_ITFM_ENABLE.ubx_key_id, CFG_ITFM_ENABLE},
  {CFG_ITFM_ANTSETTING.ubx_key_id, CFG_ITFM_ANTSETTING},
  {CFG_ITFM_ENABLE_AUX.ubx_key_id, CFG_ITFM_ENABLE_AUX},
};

bool operator<(const ubx_key_id_t & fk1, const ubx_key_id_t & fk2)
{
  return fk1.all < fk2.all;
}
}  // namespace ubx::cfg

#endif  // UBLOX_DGNSS_NODE__UBX__UBX_CFG_ITEM_HPP_
