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

#ifndef UBLOX_DGNSS_NODE__UBX__ESF__UBX_ESF_RTCM_HPP_
#define UBLOX_DGNSS_NODE__UBX__ESF__UBX_ESF_RTCM_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
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
enum mnt_alg_status_t : u1_t {ma_off = 0, ma_initializing = 1, ma_initialized0 = 2, ma_initialized1 = 3};
enum ins_init_status_t : u1_t {ins_off = 0, ins_initializing = 1, ins_initialized = 2};

struct init_status1_t
{
  union {
    x1_t all;
    struct
    {
      wt_init_status_t wt_init_status : 2;
      mnt_alg_status_t mnt_alg_status : 3;
      ins_init_status_t ins_init_status : 2;
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
      imu_init_status_t imu_init_status : 2;
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

enum calib_status_t : u1_t {not_calibrated = 0b00, calibrating = 0b01, calibrated0 = 0b01, calibrated1 = 0b11};
enum time_status_t : u1_t {ts_no_data = 0b00, ts_first_byte_used = 0b01, ts_ttag_provided = 0b11};

struct sens_status2_t
{
  union {
    x1_t all;
    struct
    {
      calib_status_t calib_status : 2;
      time_status_t time_status : 2;
    } bits;
  };
};

struct faults_t
{
  union {
    x1_t all;
    struct
    {
      bool bad_meas : 1;
      bool bad_ttag : 1;
      bool missing_meas : 1;
      bool noisy_meas : 1;
    } bits;
  };

};

struct sensor_t
{
  union {
    x4_t all;
    struct
    {
      sens_status1_t sens_status1;
      sens_status2_t sens_status2 ;
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
  init_status1_t init_status1;
  init_status2_t init_status2;
  fusion_mode_t fusion_mode;
  u1_t num_sens;    // number of sensors

 std::vector<sensor_t> sensor_statuses;

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
    init_status1 = buf_offset<init_status1_t>(&payload_, 5);
    init_status2 = buf_offset<init_status2_t>(&payload_, 6);
    fusion_mode = buf_offset<fusion_mode_t>(&payload_, 12);
    num_sens = buf_offset<u1_t>(&payload_, 15);

    // extract num_sens sensor status
    for (int i = 0; i < num_sens; i++) {
      sensor_statuses.push_back(buf_offset<sensor_t>(&payload_, 16+(i*4)));
    };
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
    oss << " wt_init: " << +init_status1.bits.wt_init_status;
    oss << " mnt_alg: " << +init_status1.bits.mnt_alg_status;
    oss << " ins_init: " << +init_status1.bits.ins_init_status;
    oss << " imu_init: " << +init_status2.bits.imu_init_status;
    oss << " fusion_mode: " << +fusion_mode;
    oss << " num_sens: " << +num_sens;
    oss << " [";

    for (int i = 0; i < num_sens; i++) {
      if (i > 0) oss << " |";
      sensor_t& sensor = sensor_statuses[i];
      oss << " type: " << +sensor.bits.sens_status1.bits.type;
      oss << " used: " << +sensor.bits.sens_status1.bits.used;
      oss << " ready: " << +sensor.bits.sens_status1.bits.ready;
      oss << " calib: " << +sensor.bits.sens_status2.bits.calib_status;
      oss << " time: " << +sensor.bits.sens_status2.bits.time_status;
      oss << " Hz: " << +sensor.bits.freq;
      oss << " bad_meas: " << +sensor.bits.faults.bits.bad_meas;
      oss << " bad_ttag: " << +sensor.bits.faults.bits.bad_meas;
      oss << " missing: " << +sensor.bits.faults.bits.missing_meas;
      oss << " noisy: " << +sensor.bits.faults.bits.noisy_meas;
    };

    oss << " ]";
    return oss.str();
  }
};
}  // namespace ubx::esf::status
#endif  // UBLOX_DGNSS_NODE__UBX__ESF__UBX_ESF_STATUS_HPP_