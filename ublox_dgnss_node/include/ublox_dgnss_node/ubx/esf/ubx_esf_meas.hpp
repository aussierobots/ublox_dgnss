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

#ifndef UBLOX_DGNSS_NODE__UBX__ESF__UBX_ESF_MEAS_HPP_
#define UBLOX_DGNSS_NODE__UBX__ESF__UBX_ESF_MEAS_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::esf::meas
{
  struct flags_t
  {
    union {
      x2_t all;
      struct
      {
        u8_t timeMarkSent : 2;
        bool timeMarkEdge : 1;
        bool calibTtagValid : 1;
        u8_t numMeas : 5;
      } bits;
    };
  };

  struct data_t
  {
    union {
      x4_t all;
      struct
      {
        u4_t dataField : 24;
        u8_t dataType : 6;
      } bits;
    };
  };


class ESFMeasPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_ESF;
  static const msg_id_t MSG_ID = UBX_ESF_MEAS;

  u4_t timeTag;        // ms - GPS Time of week of the navigation epoch.
  flags_t flags;
  u2_t id;
  std::vector<data_t> datum;
  std::vector<u4_t> calibTtags;    // number of sensors


public:
  ESFMeasPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
   ESFMeasPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    timeTag = buf_offset<u4_t>(&payload_, 0);
    flags = buf_offset<flags_t>(&payload_, 4);
    id = buf_offset<u2_t>(&payload_, 6);

    // data fields numMeas times
    auto numMeas = flags.bits.numMeas;
    datum.clear();
    for (int i = 0; i < numMeas; i++) {
      datum.push_back(buf_offset<data_t>(&payload_, 8+(i*4)));
    };

    // calibTtags fiel
    calibTtags.clear();
    // make sure the calibTTags are there
    uint calibTtags_start = 8+(numMeas*4);
    if (flags.bits.calibTtagValid && calibTtags_start+(numMeas*4) <= size) {
      for (int i = 0; i < numMeas; i++) {
        calibTtags.push_back(buf_offset<u4_t>(&payload_, calibTtags_start=(i*4)));
      };
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
    oss << "timeTag: " << +timeTag;
    oss << " timeMarkSent: " << +flags.bits.timeMarkSent;
    oss << " timeMarkEdge: " << +flags.bits.timeMarkEdge;
    oss << " calibTragValid: " << +flags.bits.calibTtagValid;
    oss << " numMeas: " << +flags.bits.numMeas;
    oss << " data: [";

    auto numMeas = flags.bits.numMeas;

    for (int i = 0; i < numMeas; i++) {
      if (i > 0) oss << " |";
      data_t& data = datum[i];
      oss << " field: " << +data.bits.dataField;
      oss << " type: " << +data.bits.dataType;
    };
    oss << " ]";

    if (flags.bits.calibTtagValid) {
      oss << " calibTtag [";
      for (int i = 0; i < numMeas; i++) {
        if (i > 0) oss << ", ";
        oss << +calibTtags[i];
      };

      oss << "]";
    }

    return oss.str();
  }
};

}  // namespace ubx::esf::meas
#endif  // UBLOX_DGNSS_NODE__UBX__ESF__UBX_ESF_MEAS_HPP_