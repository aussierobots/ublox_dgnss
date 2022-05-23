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

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_INF_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_INF_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"

namespace ubx::inf
{

class InfDebugPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_INF;
  static const msg_id_t MSG_ID = UBX_INF_DEBUG;

  std::string str;        // ASCII characters

public:
  InfDebugPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  InfDebugPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    str = std::string(reinterpret_cast<char *>(payload_.data()));
  }
  std::string to_string()
  {
    std::ostringstream oss;
    oss << "str: " << str;
    return oss.str();
  }
};


class InfErrorPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_INF;
  static const msg_id_t MSG_ID = UBX_INF_ERROR;

  std::string str;        // ASCII characters

public:
  InfErrorPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  InfErrorPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    str = std::string(reinterpret_cast<char *>(payload_.data()));
  }

  std::string to_string()
  {
    std::ostringstream oss;
    oss << "str: " << str;
    return oss.str();
  }
};


class InfNoticePayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_INF;
  static const msg_id_t MSG_ID = UBX_INF_NOTICE;

  std::string str;        // ASCII characters

public:
  InfNoticePayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  InfNoticePayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    str = std::string(reinterpret_cast<char *>(payload_.data()));
  }
  std::string to_string()
  {
    std::ostringstream oss;
    oss << "str: " << str;
    return oss.str();
  }
};

class InfTestPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_INF;
  static const msg_id_t MSG_ID = UBX_INF_TEST;

  std::string str;        // ASCII characters

public:
  InfTestPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  InfTestPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    str = std::string(reinterpret_cast<char *>(payload_.data()));
  }
  std::string to_string()
  {
    std::ostringstream oss;
    oss << "str: " << str;
    return oss.str();
  }
};

class InfWarningPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_INF;
  static const msg_id_t MSG_ID = UBX_INF_WARNING;

  std::string str;        // ASCII characters

public:
  InfWarningPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }

  InfWarningPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    str = std::string(reinterpret_cast<char *>(payload_.data()));
  }
  std::string to_string()
  {
    std::ostringstream oss;
    oss << "str: " << str;
    return oss.str();
  }
};


typedef UBXFrameComms<inf::InfDebugPayload, usb::Connection> UbxInfDebugFrameComms;
typedef UBXFrameComms<inf::InfErrorPayload, usb::Connection> UbxInfErrorFrameComms;
typedef UBXFrameComms<inf::InfNoticePayload, usb::Connection> UbxInfNoticeFrameComms;
typedef UBXFrameComms<inf::InfTestPayload, usb::Connection> UbxInfTestFrameComms;
typedef UBXFrameComms<inf::InfWarningPayload, usb::Connection> UbxInfWarningFrameComms;

class UbxInf
{
private:
  std::shared_ptr<usb::Connection> usbc_;

  std::shared_ptr<UbxInfDebugFrameComms> debug_;
  std::shared_ptr<UbxInfErrorFrameComms> error_;
  std::shared_ptr<UbxInfNoticeFrameComms> notice_;
  std::shared_ptr<UbxInfTestFrameComms> test_;
  std::shared_ptr<UbxInfWarningFrameComms> warning_;

public:
  explicit UbxInf(std::shared_ptr<usb::Connection> usbc)
  {
    usbc_ = usbc;
    debug_ = std::make_shared<UbxInfDebugFrameComms>(usbc_);
    error_ = std::make_shared<UbxInfErrorFrameComms>(usbc_);
    notice_ = std::make_shared<UbxInfNoticeFrameComms>(usbc_);
    test_ = std::make_shared<UbxInfTestFrameComms>(usbc_);
    warning_ = std::make_shared<UbxInfWarningFrameComms>(usbc_);
  }
  std::shared_ptr<UbxInfDebugFrameComms> debug()
  {
    return debug_;
  }
  std::shared_ptr<UbxInfErrorFrameComms> error()
  {
    return error_;
  }
  std::shared_ptr<UbxInfNoticeFrameComms> notice()
  {
    return notice_;
  }
  std::shared_ptr<UbxInfTestFrameComms> test()
  {
    return test_;
  }
  std::shared_ptr<UbxInfWarningFrameComms> warning()
  {
    return warning_;
  }
  void frame(std::shared_ptr<ubx::Frame> frame)
  {
    switch (frame->msg_id) {
      case ubx::UBX_INF_DEBUG:
        debug_->frame(frame);
        break;
      case ubx::UBX_INF_ERROR:
        error_->frame(frame);
        break;
      case ubx::UBX_INF_NOTICE:
        notice_->frame(frame);
        break;
      case ubx::UBX_INF_TEST:
        test_->frame(frame);
        break;
      case ubx::UBX_INF_WARNING:
        warning_->frame(frame);
        break;
      default:
        // break;
        throw UbxValueException("unknown UBX_INF msg_id: " + ubx::to_hex(frame->msg_id));
    }
  }
};
}  // namespace ubx::inf
#endif  // UBLOX_DGNSS_NODE__UBX__UBX_INF_HPP_
