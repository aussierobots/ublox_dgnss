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
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"
#include "ublox_ubx_msgs/msg/ubx_esf_meas.hpp"

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
  u4_t calibTtags;    // number of sensors

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
    uint numMeas = static_cast<uint>(flags.bits.numMeas);
    datum.clear();
    for (uint i = 0; i < numMeas; i++) {
      datum.push_back(buf_offset<data_t>(&payload_, 8 + (i * 4)));
    }

    // make sure the calibTTags are there
    if (flags.bits.calibTtagValid) {
      uint calibTtags_start = 8 + (numMeas * 4);
      calibTtags = buf_offset<u4_t>(&payload_, calibTtags_start);
    }
  }

  // this payload is used to poll for output from device
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

    uint numMeas = static_cast<uint>(flags.bits.numMeas);

    for (uint i = 0; i < numMeas; i++) {
      if (i > 0) {oss << " |";}
      data_t & data = datum[i];
      oss << " field: " << +data.bits.dataField;
      oss << " type: " << +data.bits.dataType;
    }
    oss << " ]";
    oss << +calibTtags;
    if (flags.bits.calibTtagValid) {
      oss << " " << +calibTtags;
    }

    return oss.str();
  }
};

// this is the full payload - ubx_esf_meas has input & output
// it is an exception to UBX where other cfg items are set via val set
// it uses the local values to create the full payload
class ESFMeasFullPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_ESF;
  static const msg_id_t MSG_ID = UBX_ESF_MEAS;

  u4_t timeTag;        // ms - GPS Time of week of the navigation epoch.
  flags_t flags;
  u2_t id;
  std::vector<data_t> datum;
  u4_t calibTtag;    // number of sensors

public:
  ESFMeasFullPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  void load_from_msg(const ublox_ubx_msgs::msg::UBXEsfMeas & msg)
  {
    // assign number of measurements if 0 use data array size
    u8_t num_meas = msg.num_meas;
    if (num_meas == 0) {
      num_meas = msg.data.size();
    }
    timeTag = msg.time_tag;
    flags.bits.timeMarkSent = msg.time_mark_sent;
    flags.bits.timeMarkEdge = msg.time_mark_edge;
    flags.bits.calibTtagValid = msg.calib_ttag_valid;
    flags.bits.numMeas = num_meas;
    id = msg.id;

    data_t data;
    datum.clear();
    for (uint i = 0; i < msg.data.size(); i++) {
      data.bits.dataField = msg.data[i].data_field;
      data.bits.dataType = msg.data[i].data_type;
      datum.push_back(data);
    }

    if (msg.calib_ttag_valid) {
      calibTtag = msg.calib_ttag;
    } else {
      calibTtag = 0;
    }
  }
  // this payload is used as input to the device
  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();

    buf_append_u4(&payload_, timeTag);
    buf_append_x2(&payload_, flags.all);
    buf_append_u2(&payload_, id);

    auto numMeas = flags.bits.numMeas;
    for (uint i = 0; i < numMeas; i++) {
      buf_append_x4(&payload_, datum[i].all);
    }

    if (flags.bits.calibTtagValid) {
      buf_append_u4(&payload_, calibTtag);
    }

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

    uint numMeas = static_cast<uint>(flags.bits.numMeas);

    for (uint i = 0; i < numMeas; i++) {
      if (i > 0) {oss << " |";}
      data_t & data = datum[i];
      oss << " field: " << +data.bits.dataField;
      oss << " type: " << +data.bits.dataType;
    }
    oss << " ]";

    if (flags.bits.calibTtagValid) {
      oss << " calibTtag: " << +calibTtag;
    }

    return oss.str();
  }
};

}  // namespace ubx::esf::meas
#endif  // UBLOX_DGNSS_NODE__UBX__ESF__UBX_ESF_MEAS_HPP_
