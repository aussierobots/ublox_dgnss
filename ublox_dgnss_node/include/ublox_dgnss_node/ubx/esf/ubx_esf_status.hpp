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

#ifndef UBLOX_DGNSS_NODE__UBX__ESF__UBX_ESF_STATUS_HPP_
#define UBLOX_DGNSS_NODE__UBX__ESF__UBX_ESF_STATUS_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::esf::status
{

enum msg_used_t : u1_t {rtcm_used = 2, rtcm_failed = 1, rtcm_unknown = 0};

struct flags_t
{
  union {
    x1_t all;
    struct
    {
      l_t crcFailed : 1;
      msg_used_t msgUsed : 2;
    } bits;
  };
};

enum wt_init_status_t : u1_t {wt_off = 0, wt_initializing = 1, wt_initialized = 2};
enum mnt_alg_status_t : u1_t {ma_off = 0, ma_initializing = 1, ma_initialized0 = 2,
  ma_initialized1 = 3};
enum ins_init_status_t : u1_t {ins_off = 0, ins_initializing = 1, ins_initialized = 2};

struct init_status1_t
{
  union {
    x1_t all;
    struct
    {
      wt_init_status_t wtInitStatus : 2;
      mnt_alg_status_t mntAlgStatus : 3;
      ins_init_status_t insInitStatus : 2;
    } bits;
  };
};

enum imu_init_status_t : u1_t {imu_off = 0, imu_initializing = 1, imu_initialized = 2};

struct init_status2_t
{
  union {
    x1_t all;
    struct
    {
      imu_init_status_t imuInitStatus : 2;
    } bits;
  };
};

enum  fusion_mode_t : u1_t {fusion_initialization = 0, fusion_working = 1,
  fusion_suspended = 2, fusion_disabled = 3};

struct sens_status1_t
{
  union {
    x1_t all;
    struct
    {
      u1_t type : 6;
      bool used : 1;
      bool ready : 1;
    } bits;
  };
};

enum calib_status_t : u1_t {not_calibrated = 0b00, calibrating = 0b01, calibrated0 = 0b01,
  calibrated1 = 0b11};
enum time_status_t : u1_t {ts_no_data = 0b00, ts_first_byte_used = 0b01, ts_ttag_provided = 0b11};

struct sens_status2_t
{
  union {
    x1_t all;
    struct
    {
      calib_status_t calibStatus : 2;
      time_status_t timeStatus : 2;
    } bits;
  };
};

struct faults_t
{
  union {
    x1_t all;
    struct
    {
      bool badMeas : 1;
      bool badTtag : 1;
      bool missingMeas : 1;
      bool noisyMeas : 1;
    } bits;
  };
};

struct sensor_t
{
  union {
    x4_t all;
    struct
    {
      sens_status1_t sensStatus1;
      sens_status2_t sensStatus2;
      u1_t freq;
      faults_t faults;
    } bits;
  };
};

class ESFStatusPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_ESF;
  static const msg_id_t MSG_ID = UBX_ESF_STATUS;

  u4_t iTOW;        // ms - GPS Time of week of the navigation epoch.
  u1_t version;     // message version (0x02 for this version)
  init_status1_t initStatus1;
  init_status2_t initStatus2;
  fusion_mode_t fusionMode;
  u1_t numSens;    // number of sensors

  std::vector<sensor_t> sensorStatuses;

public:
  ESFStatusPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  ESFStatusPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    iTOW = buf_offset<u4_t>(&payload_, 0);
    version = buf_offset<u1_t>(&payload_, 4);
    initStatus1 = buf_offset<init_status1_t>(&payload_, 5);
    initStatus2 = buf_offset<init_status2_t>(&payload_, 6);
    fusionMode = buf_offset<fusion_mode_t>(&payload_, 12);
    numSens = buf_offset<u1_t>(&payload_, 15);

    // extract num_sens sensor status
    sensorStatuses.clear();
    for (int i = 0; i < numSens; i++) {
      sensorStatuses.push_back(buf_offset<sensor_t>(&payload_, 16 + (i * 4)));
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
    oss << "iTOW: " << iTOW;
    oss << " version: " << +version;
    oss << " wtInit: " << +initStatus1.bits.wtInitStatus;
    oss << " mntAlg: " << +initStatus1.bits.mntAlgStatus;
    oss << " insInit: " << +initStatus1.bits.insInitStatus;
    oss << " imuInit: " << +initStatus2.bits.imuInitStatus;
    oss << " fusionMode: " << +fusionMode;
    oss << " numSens: " << +numSens;
    oss << " [";

    for (int i = 0; i < numSens; i++) {
      if (i > 0) {oss << " |";}
      sensor_t & sensor = sensorStatuses[i];
      oss << " type: " << +sensor.bits.sensStatus1.bits.type;
      oss << " used: " << +sensor.bits.sensStatus1.bits.used;
      oss << " ready: " << +sensor.bits.sensStatus1.bits.ready;
      oss << " calib: " << +sensor.bits.sensStatus2.bits.calibStatus;
      oss << " time: " << +sensor.bits.sensStatus2.bits.timeStatus;
      oss << " Hz: " << +sensor.bits.freq;
      oss << " badMeas: " << +sensor.bits.faults.bits.badMeas;
      oss << " badTtag: " << +sensor.bits.faults.bits.badMeas;
      oss << " missing: " << +sensor.bits.faults.bits.missingMeas;
      oss << " noisy: " << +sensor.bits.faults.bits.noisyMeas;
    }

    oss << " ]";
    return oss.str();
  }
};
}  // namespace ubx::esf::status
#endif  // UBLOX_DGNSS_NODE__UBX__ESF__UBX_ESF_STATUS_HPP_
