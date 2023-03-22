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

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_HPP_
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cstring>
#include <string>
#include <tuple>
#include <memory>
#include "ublox_dgnss_node/usb.hpp"
#include "ublox_dgnss_node/ubx/ubx_types.hpp"
#include "ublox_dgnss_node/ubx/ubx_msg.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"
#include "ublox_dgnss_node/ubx/ubx_exceptions.hpp"

namespace ubx
{

struct Frame
{
  u1_t sync_char_1 = UBX_SYNC_CHAR_1;
  u1_t sync_char_2 = UBX_SYNC_CHAR_2;
  msg_class_t msg_class;
  msg_id_t msg_id;
  u2_t length;       // of payload
  ch_t * payload;
  u1_t ck_a;
  u1_t ck_b;
  std::vector<u1_t> buf;
  u1_t * build_frame_buf()
  {
    buf.clear();
    buf.reserve(8 + length);
    buf.push_back(sync_char_1);
    buf.push_back(sync_char_2);
    buf.push_back(msg_class);
    buf.push_back(msg_id);
    buf_append_u2(&buf, length);
    for (int i = 0; i < length; i++) {
      buf.push_back(payload[i]);
    }
    // put markers in for checksom values
    buf.push_back(ck_a);
    buf.push_back(ck_b);
    return &buf[0];
  }

  void from_buf_build()
  {
    sync_char_1 = buf[0];
    sync_char_2 = buf[1];
    msg_class = buf[2];
    msg_id = buf[3];
    length = *reinterpret_cast<u2_t *>(&buf[4]);
    payload = reinterpret_cast<ch_t *>(&buf[6]);
    ck_a = buf[buf.size() - 2];
    ck_b = buf[buf.size() - 1];
  }

  std::tuple<u1_t, u1_t> ubx_check_sum()
  {
    build_frame_buf();
    size_t n = buf.size() - 2;             // dont include the two checksum fields
    u1_t ck_a = 0;
    u1_t ck_b = 0;
    for (size_t i = 2; i < n; i++) {
      ck_a = ck_a + buf[i];
      ck_b = ck_b + ck_a;
    }

    return std::make_tuple(ck_a, ck_b);
  }
  std::string to_hex()
  {
    std::ostringstream os;
    os << "size: " << buf.size() << " '0x";
    for (const auto & u1 : buf) {
      os << ubx::to_hex(u1);
    }
    os << "'";
    return os.str();
  }
  std::string payload_to_hex()
  {
    std::ostringstream os;
    os << "length: " << length << " '0x";
    for (size_t i = 0; i < length; i++) {
      const u1_t u1 = (u1_t)payload[i];
      os << ubx::to_hex(u1);
    }
    os << "'";
    return os.str();
  }
};

using FramePoll = Frame;
using FramePolled = Frame;
using FrameValSet = Frame;

std::shared_ptr<FramePolled> get_polled_frame(
  std::shared_ptr<usb::Connection> usbc,
  std::shared_ptr<FramePoll> poll_frame)
{
  usbc->write_buffer(&poll_frame->buf[0], poll_frame->buf.size());

  auto polled_frame = std::make_shared<FramePolled>();

  int max_retries = 1000 / usbc->timeout_ms();       // max retries per second
  int i = 0;
  static u_char buf[64 * 100 + 1];
  int len;
  do {
    std::memset(buf, 0, sizeof(buf));
    try {
      len = usbc->read_chars(buf, sizeof(buf));
    } catch (usb::UsbException & e) {
      throw e;
    } catch (usb::TimeoutException & e) {
      // timeout is set in usbc_
      continue;
    } catch (std::exception & e) {
      throw e;
    }


    if (len > 0) {
      if (buf[0] == UBX_SYNC_CHAR_1 && buf[1] == UBX_SYNC_CHAR_2) {
        polled_frame->buf.resize(len);
        memcpy(&polled_frame->buf[0], &buf[0], len);
        polled_frame->from_buf_build();

        // make sure checksums match
        u1_t ck_a, ck_b;
        std::tie(ck_a, ck_b) = polled_frame->ubx_check_sum();
        if (ck_a != polled_frame->ck_a && ck_b != polled_frame->ck_b) {
          throw UbxAckNackException("polled frame checksum failed");
        }

        // if device failed to acknowledge the poll request then Nak it
        if (polled_frame->msg_class == UBX_ACK && polled_frame->msg_id == UBX_ACK_NAK) {
          std::ostringstream msg_oss;
          msg_oss << "UBX_ACK_NAK fail";
          msg_oss << " sent poll_frame.msg_class: " << "0x" << std::setfill('0') << std::setw(2) <<
            std::right << std::hex << +poll_frame->msg_class;
          msg_oss << " poll_frame.msg_id: " << "0x" << std::setfill('0') << std::setw(2) <<
            std::right << std::hex << +poll_frame->msg_id;
          msg_oss << " repsonse polled_frame.msg_class: " << "0x" << std::setfill('0') << std::setw(
            2) << std::right << std::hex << +polled_frame->msg_class;
          msg_oss << " polled_frame.msg_id: " << "0x" << std::setfill('0') << std::setw(2) <<
            std::right << std::hex << +polled_frame->msg_id;
          throw UbxAckNackException(msg_oss.str());
        }

        // exit while loop
        break;
      }
    }
  } while (++i < max_retries);

  if (i >= max_retries) {
    std::ostringstream msg_oss;
    msg_oss << "UBX_ACK_NAK wasnt received after " << i << " tries";
    msg_oss << " sent poll_frame.msg_class: " << "0x" << std::setfill('0') << std::setw(2) <<
      std::right << std::hex << +poll_frame->msg_class;
    msg_oss << " poll_frame.msg_id: " << "0x" << std::setfill('0') << std::setw(2) << std::right <<
      std::hex << +poll_frame->msg_id;

    throw UbxAckNackException(msg_oss.str());
  }

  return polled_frame;
}

class UBXPayloadBase
{
protected:
  msg_class_t msg_class_;
  msg_id_t msg_id_;
  std::vector<u1_t> payload_;

  UBXPayloadBase(msg_class_t msg_class, msg_id_t msg_id)
  : msg_class_(msg_class), msg_id_(msg_id)
  {
    payload_.clear();
  }
  UBXPayloadBase()
  : msg_class_(-1), msg_id_(-1)
  {
  }
};

class UBXPayload : UBXPayloadBase
{
protected:
  msg_class_t msg_class_;
  msg_id_t msg_id_;
  std::vector<u1_t> payload_;

  UBXPayload(msg_class_t msg_class, msg_id_t msg_id)
  : UBXPayloadBase(msg_class, msg_id), msg_class_(msg_class), msg_id_(msg_id)
  {
    payload_.clear();
  }
  UBXPayload()
  : msg_class_(-1), msg_id_(-1)
  {
  }

public:
  virtual std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }
};

class UBXPayloadPoll : UBXPayloadBase
{
public:
  UBXPayloadPoll()
  : UBXPayloadBase(-1, -1)
  {
  }
  UBXPayloadPoll(msg_class_t msg_class, msg_id_t msg_id)
  : UBXPayloadBase(msg_class, msg_id)
  {
  }
// zero length payload
  msg_class_t msg_class()
  {
    return msg_class_;
  }
  msg_id_t msg_id()
  {
    return msg_id_;
  }
};


class UBXPayloadOutputPrint : UBXPayloadBase
{
public:
  UBXPayloadOutputPrint()
  : UBXPayloadBase(-1, -1)
  {
  }
  UBXPayloadOutputPrint(msg_class_t msg_class, msg_id_t msg_id)
  : UBXPayloadBase(msg_class, msg_id)
  {
  }

  std::string to_hex()
  {
    u1_t * buf = payload_.data();
    size_t size = payload_.size();
    std::ostringstream os;

    os << "size: " << size << " '0x";
    for (size_t i = 0; i < size; i++) {
      os << std::setfill('0') << std::setw(2) << std::right << std::hex << +buf[i];
    }
    os << "'";
    return os.str();
  }
};

template<typename T>
class Payload : public T, public UBXPayloadOutputPrint
{
public:
  Payload(ch_t * payload_polled, u2_t size)
  : T(payload_polled, size), UBXPayloadOutputPrint(T::MSG_CLASS, T::MSG_ID)
  {
  }
  Payload()
  : T(), UBXPayloadOutputPrint(T::MSG_CLASS, T::MSG_ID)
  {
  }
};

template<typename T>
class PayloadPoll : public T, public UBXPayloadPoll
{
public:
  PayloadPoll()
  : T(), UBXPayloadPoll(T::MSG_CLASS, T::MSG_ID)
  {
  }
};


template<typename T>
class FrameContainer
{
private:
  msg_class_t msg_class_;
  msg_id_t msg_id_;
  std::shared_ptr<Frame> frame_;
  std::shared_ptr<FramePoll> frame_poll_;
  std::shared_ptr<Payload<T>> payload_;
  std::shared_ptr<PayloadPoll<T>> payload_poll_;

public:
  FrameContainer()
  {
    payload_poll_ = std::make_shared<PayloadPoll<T>>();
    payload_ = std::make_shared<Payload<T>>();
    this->msg_class_ = payload_poll_->msg_class();
    this->msg_id_ = payload_poll_->msg_id();
  }

  msg_class_t msg_class()
  {
    return msg_class_;
  }
  msg_id_t msg_id()
  {
    return msg_id_;
  }
  std::shared_ptr<Frame> frame()
  {
    return frame_;
  }
  std::shared_ptr<FramePoll> frame_poll()
  {
    if (frame_poll_.use_count() == 0) {
      make_frame_poll();
    }
    return frame_poll_;
  }
  std::shared_ptr<Payload<T>> payload()
  {
    return payload_;
  }
  std::shared_ptr<PayloadPoll<T>> payload_poll()
  {
    return payload_poll_;
  }

  std::shared_ptr<ubx::FramePoll> make_frame_poll()
  {
    u1_t * payload;
    size_t payload_size;

    if (payload_poll_.use_count() == 0) {
      throw UbxPayloadException("No poll payload set!");
    }
    std::tie(payload, payload_size) = payload_poll_->make_poll_payload();

    frame_poll_ = std::make_shared<ubx::FramePoll>();
    frame_poll_->msg_class = T::MSG_CLASS;
    frame_poll_->msg_id = T::MSG_ID;
    frame_poll_->payload = reinterpret_cast<ch_t *>(payload);
    frame_poll_->length = payload_size;
    std::tie(frame_poll_->ck_a, frame_poll_->ck_b) = frame_poll_->ubx_check_sum();
    frame_poll_->build_frame_buf();

    return frame_poll_;
  }

  void frame(std::shared_ptr<Frame> frame)
  {
    if (frame->msg_class != msg_class_ || frame->msg_id != msg_id_) {
      throw UbxValueException("msg class & id for frame dont match frame type's");
    }
    frame_ = frame;
    payload_ = std::make_shared<Payload<T>>(frame->payload, frame->length);
  }
};

template<class T, class U>
class UBXFrameComms
{
private:
  std::shared_ptr<FrameContainer<T>> container_;
  std::shared_ptr<U> comms_;

public:
  explicit UBXFrameComms(std::shared_ptr<U> comms)
  {
    comms_ = comms;
    container_ = std::make_shared<FrameContainer<T>>();
  }
  std::shared_ptr<ubx::Frame> frame()
  {
    return container_->frame();
  }
  std::shared_ptr<Payload<T>> payload()
  {
    return container_->payload();
  }
  void frame(std::shared_ptr<ubx::Frame> frame)
  {
    container_->frame(frame);
  }
  void poll_async()
  {
    auto frame_poll = container_->frame_poll();
    comms_->write_buffer_async(frame_poll->buf.data(), frame_poll->buf.size(), NULL);
  }
};
}  // namespace ubx

#endif  // UBLOX_DGNSS_NODE__UBX__UBX_HPP_
