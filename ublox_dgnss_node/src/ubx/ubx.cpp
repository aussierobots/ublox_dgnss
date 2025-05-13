// Copyright 2025 GreenForge Labs
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

/**
 * @file ubx.cpp
 * @brief Implementation of UBX protocol functions
 */

#include "ublox_dgnss_node/ubx/ubx.hpp"
#include <sstream>
#include <iomanip>
#include <cstring>

namespace ubx
{

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

}  // namespace ubx
