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
 * @file usb_ubx_transceiver.cpp
 * @brief Implementation of the UsbUbxTransceiver class for USB-based UBX message communication
 */

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <libusb-1.0/libusb.h>

#include "ublox_dgnss_node/ubx/usb_ubx_transceiver.hpp"
#include "ublox_dgnss_node/ubx/ubx_ack.hpp"

namespace ubx
{

UsbUbxTransceiver::UsbUbxTransceiver(
  std::shared_ptr<usb::Connection> usbc,
  rclcpp::Logger logger)
: usbc_(usbc),
  logger_(logger)
{
}

bool UsbUbxTransceiver::send_ubx_message(const std::shared_ptr<ubx::Frame> & frame)
{
  if (!is_connected()) {
    RCLCPP_ERROR(logger_, "Cannot send message: USB connection is not open");
    return false;
  }

  try {
    // Calculate checksum and build frame buffer
    std::tie(frame->ck_a, frame->ck_b) = frame->ubx_check_sum();
    frame->build_frame_buf();

    // Send the message
    int bytes_written = usbc_->write_buffer(frame->buf.data(), frame->buf.size());

    if (bytes_written != static_cast<int>(frame->buf.size())) {
      RCLCPP_ERROR(
        logger_,
        "Failed to send complete message: wrote %d of %zu bytes",
        bytes_written, frame->buf.size());
      return false;
    }

    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_,
      "Error sending UBX message: %s", e.what());
    return false;
  }
}

int UsbUbxTransceiver::receive_ubx_message(std::vector<u1_t> & buffer, int timeout_ms)
{
  if (!is_connected()) {
    RCLCPP_ERROR(logger_, "Cannot receive message: USB connection is not open");
    return -1;
  }

  try {
    // Ensure buffer is large enough
    if (buffer.size() < 1024) {
      buffer.resize(1024);
    }

    // Read with timeout
    auto start_time = std::chrono::steady_clock::now();
    int total_bytes_read = 0;
    int bytes_read = 0;

    // Read data until we get a complete UBX message or timeout
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time).count() < timeout_ms)
    {
      bytes_read = usbc_->read_chars(buffer.data() + total_bytes_read,
                                     buffer.size() - total_bytes_read);

      if (bytes_read > 0) {
        total_bytes_read += bytes_read;

        // Check if we have a complete UBX message
        size_t i = 0;
        while (i < static_cast<size_t>(total_bytes_read - 1)) {
          if (buffer[i] == 0xB5 && buffer[i + 1] == 0x62) {  // UBX sync chars
            // Check if we have enough bytes for a complete message
            if (i + 8 <= static_cast<size_t>(total_bytes_read)) {  // Header is 6 bytes, plus at least 2 for checksum
              u2_t length = *reinterpret_cast<u2_t *>(&buffer[i + 4]);

              // Check if we have the complete message
              if (i + 8 + length <= static_cast<size_t>(total_bytes_read)) {
                // Move the complete message to the beginning of the buffer
                if (i > 0) {
                  std::copy(buffer.begin() + i, buffer.begin() + i + 8 + length, buffer.begin());
                  total_bytes_read = 8 + length;
                }

                return total_bytes_read;
              }
            }
          }
          i++;
        }

        // If we didn't find a complete message, continue reading
      } else if (bytes_read == 0) {
        // No data available, sleep a bit to avoid busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      } else {
        // Error reading
        RCLCPP_ERROR(logger_, "Error reading from USB connection");
        return -1;
      }
    }

    // Timeout
    if (total_bytes_read > 0) {
      RCLCPP_WARN(
        logger_,
        "Timeout waiting for complete UBX message, received %d bytes",
        total_bytes_read);
    } else {
      RCLCPP_ERROR(logger_, "Timeout waiting for UBX message, no data received");
    }

    return total_bytes_read;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_,
      "Error receiving UBX message: %s", e.what());
    return -1;
  }
}

bool UsbUbxTransceiver::wait_for_ack(uint8_t msg_class, uint8_t msg_id, int timeout_ms)
{
  if (!is_connected()) {
    RCLCPP_ERROR(logger_, "Cannot wait for ACK: USB connection is not open");
    return false;
  }

  try {
    std::vector<u1_t> buffer(1024);
    auto start_time = std::chrono::steady_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time).count() < timeout_ms)
    {
      int bytes_read = receive_ubx_message(buffer, 100);  // Use shorter timeout for each read

      if (bytes_read > 0) {
        // Check if this is an ACK-ACK or ACK-NAK message
        if (buffer[2] == 0x05 && (buffer[3] == 0x01 || buffer[3] == 0x00)) {
          // Parse the ACK-ACK or ACK-NAK message
          ubx::ack::AckNakPayload ack_nak(&buffer[6], 2);  // Skip header (6 bytes) to get to payload

          // Check if this ACK/NAK is for our message
          if (ack_nak.cls_id == msg_class && ack_nak.msg_id == msg_id) {
            // ACK-ACK has ID 0x01, ACK-NAK has ID 0x00
            bool is_ack = (buffer[3] == 0x01);

            if (is_ack) {
              RCLCPP_DEBUG(
                logger_,
                "Received ACK for message 0x%02X 0x%02X",
                msg_class, msg_id);
            } else {
              RCLCPP_WARN(
                logger_,
                "Received NAK for message 0x%02X 0x%02X",
                msg_class, msg_id);
            }

            return is_ack;
          }
        }
      } else if (bytes_read < 0) {
        // Error reading
        return false;
      }

      // No relevant message yet, continue waiting
    }

    // Timeout
    RCLCPP_ERROR(
      logger_,
      "Timeout waiting for ACK/NAK for message 0x%02X 0x%02X",
      msg_class, msg_id);
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_,
      "Error waiting for ACK: %s", e.what());
    return false;
  }
}

bool UsbUbxTransceiver::is_connected()
{
  return usbc_ && usbc_->is_open();
}

bool UsbUbxTransceiver::connect()
{
  if (!usbc_) {
    RCLCPP_ERROR(logger_, "USB connection object is null");
    return false;
  }

  if (usbc_->is_open()) {
    RCLCPP_DEBUG(logger_, "USB connection is already open");
    return true;
  }

  bool result = usbc_->open();
  if (result) {
    RCLCPP_INFO(logger_, "USB connection opened successfully");
  } else {
    RCLCPP_ERROR(logger_, "Failed to open USB connection");
  }

  return result;
}

void UsbUbxTransceiver::disconnect()
{
  if (usbc_ && usbc_->is_open()) {
    usbc_->close();
    RCLCPP_INFO(logger_, "USB connection closed");
  }
}

}  // namespace ubx
