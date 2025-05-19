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
 * @file ubx_transceiver_factory.cpp
 * @brief Implementation of the UbxTransceiverFactory class
 */

#include "ublox_dgnss_node/ubx/ubx_transceiver_factory.hpp"
#include "ublox_dgnss_node/ubx/ubx_ack.hpp"

namespace ubx
{

/**
 * @brief Real implementation of UbxTransceiver for USB communication
 * 
 * This class provides a complete implementation of the UbxTransceiver
 * interface for communicating with u-blox devices over USB.
 */
class RealUsbTransceiver : public UbxTransceiver
{
public:
  RealUsbTransceiver(std::shared_ptr<usb::Connection> usbc, rclcpp::Logger logger)
  : usbc_(usbc), logger_(logger) {}

  WriteResult write(std::shared_ptr<const ubx::Frame> frame) override {
    WriteResult result;
    
    if (!is_open()) {
      RCLCPP_ERROR(logger_, "Cannot send message: USB connection is not open");
      result.status = AckNack::NONE;
      return result;
    }

    try {
      // Build a non-const copy of the frame that we can send
      auto sendable_frame = std::make_shared<ubx::Frame>();
      sendable_frame->msg_class = frame->msg_class;
      sendable_frame->msg_id = frame->msg_id;
      sendable_frame->length = frame->length;
      
      // Copy payload if it exists
      if (frame->length > 0 && frame->payload != nullptr) {
        // Allocate memory for the payload
        u_char* payload_copy = new u_char[frame->length];
        std::memcpy(payload_copy, frame->payload, frame->length);
        sendable_frame->payload = reinterpret_cast<ubx::ch_t*>(payload_copy);
      } else {
        sendable_frame->payload = nullptr;
        sendable_frame->length = 0;
      }
      
      // Calculate checksum and build frame buffer
      std::tie(sendable_frame->ck_a, sendable_frame->ck_b) = sendable_frame->ubx_check_sum();
      sendable_frame->build_frame_buf();

      // Send the message
      size_t bytes_to_write = sendable_frame->buf.size();
      size_t bytes_written = 0;
      
      // Actually write the data
      if (usbc_) {
        usbc_->write_buffer(sendable_frame->buf.data(), bytes_to_write);
        bytes_written = bytes_to_write; // write_buffer doesn't return number of bytes written, it throws on error
      }

      if (bytes_written != bytes_to_write) {
        RCLCPP_ERROR(
          logger_,
          "Failed to send complete message: wrote %zu of %zu bytes",
          bytes_written, bytes_to_write);
        result.status = AckNack::NONE;
        return result;
      }

      // Wait for ACK/NAK
      bool acked = wait_for_ack(frame->msg_class, frame->msg_id);
      result.status = acked ? AckNack::ACK : AckNack::NACK;
      return result;
      
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_,
        "Error sending UBX message: %s", e.what());
      result.status = AckNack::NONE;
      return result;
    }
  }

  ReadResult read(std::shared_ptr<ubx::Frame> & frame, int timeout_ms) override {
    ReadResult result;
    
    if (!is_open()) {
      RCLCPP_ERROR(logger_, "Cannot receive message: USB connection is not open");
      result.status = ReadStatus::ERROR;
      return result;
    }

    try {
      // Prepare buffer for reading
      std::vector<u_char> buffer(1024, 0);  // Use a reasonable buffer size
      
      // Read with timeout
      auto start_time = std::chrono::steady_clock::now();
      int total_bytes_read = 0;
      int bytes_read = 0;
      
      // Read header first (sync chars + class + id + length = 6 bytes)
      const int HEADER_SIZE = 6;
      
      // Keep reading until we get a complete message or timeout
      while (std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time).count() < timeout_ms)
      {
        // Check if we have a USB connection
        if (!usbc_) {
          result.status = ReadStatus::ERROR;
          return result;
        }
        
        // Use the read_chars method from the Connection class
        bytes_read = usbc_->read_chars(buffer.data() + total_bytes_read, 
                                     buffer.size() - total_bytes_read);
        
        if (bytes_read > 0) {
          total_bytes_read += bytes_read;
          
          // Continue reading until we have at least a header
          if (total_bytes_read < HEADER_SIZE) {
            continue;
          }
          
          // Find the start of a UBX message
          int start_idx = 0;
          while (start_idx <= total_bytes_read - 2) {
            if (buffer[start_idx] == 0xB5 && buffer[start_idx + 1] == 0x62) {
              break;  // Found UBX marker
            }
            start_idx++;
          }
          
          // If no marker found, or not enough bytes for a header, continue reading
          if (start_idx > total_bytes_read - HEADER_SIZE) {
            continue;
          }
          
          // Get message length from header (little endian, 2 bytes)
          uint16_t payload_length = buffer[start_idx + 4] | (buffer[start_idx + 5] << 8);
          
          // Calculate total message size (header + payload + checksum)
          uint32_t msg_size = HEADER_SIZE + payload_length + 2;
          
          // If we have a complete message, process it
          if (static_cast<uint32_t>(total_bytes_read) >= start_idx + msg_size) {
            // Create frame if it doesn't exist
            if (!frame) {
              frame = std::make_shared<ubx::Frame>();
            }
            
            // Transfer data to frame buffer
            frame->buf.resize(msg_size);
            std::copy(buffer.begin() + start_idx, 
                     buffer.begin() + start_idx + msg_size, 
                     frame->buf.begin());
            
            // Extract message components from buffer
            // Header is already verified (0xB5 0x62)
            frame->msg_class = buffer[start_idx + 2];
            frame->msg_id = buffer[start_idx + 3];
            frame->length = buffer[start_idx + 4] | (buffer[start_idx + 5] << 8);
            
            // Allocate and copy payload
            if (frame->length > 0) {
              // Allocate memory for the payload
              u_char* payload_buffer = new u_char[frame->length];
              std::copy(buffer.begin() + start_idx + 6,
                       buffer.begin() + start_idx + 6 + frame->length,
                       payload_buffer);
              frame->payload = reinterpret_cast<ubx::ch_t*>(payload_buffer);
            } else {
              frame->payload = nullptr;
            }
            
            // Extract checksum
            frame->ck_a = buffer[start_idx + 6 + frame->length];
            frame->ck_b = buffer[start_idx + 6 + frame->length + 1];
            
            // Copy the entire message to the frame buffer for future reference
            frame->buf.resize(msg_size);
            std::copy(buffer.begin() + start_idx,
                     buffer.begin() + start_idx + msg_size,
                     frame->buf.begin());
            
            // Recalculate checksum to verify message integrity
            uint8_t calc_ck_a = 0, calc_ck_b = 0;
            for (size_t i = 0; i < static_cast<size_t>(frame->length) + 4; i++) {
              calc_ck_a += buffer[start_idx + 2 + i];
              calc_ck_b += calc_ck_a;
            }
            
            if (calc_ck_a == frame->ck_a && calc_ck_b == frame->ck_b) {
              result.status = ReadStatus::SUCCESS;
              return result;
            } else {
              // Free allocated memory on error
              if (frame->payload != nullptr) {
                delete[] reinterpret_cast<u_char*>(frame->payload);
                frame->payload = nullptr;
              }
              RCLCPP_ERROR(logger_, "Checksum verification failed for UBX message");
              result.status = ReadStatus::ERROR;
              return result;
            }
          }
        } else if (bytes_read == 0) {
          // No data available, continue waiting
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else {
          // Error reading
          RCLCPP_ERROR(logger_, "Error reading from USB");
          result.status = ReadStatus::ERROR;
          return result;
        }
      }
      
      // Timeout
      RCLCPP_DEBUG(logger_, "Timeout reading UBX message after %d ms", timeout_ms);
      result.status = ReadStatus::TIMEOUT;
      return result;
      
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Error reading UBX message: %s", e.what());
      result.status = ReadStatus::ERROR;
      return result;
    }
  }

  bool is_open() override {
    return usbc_ != nullptr && usbc_->devh_valid();
  }

  bool open() override {
    if (!usbc_) {
      RCLCPP_ERROR(logger_, "USB connection is null");
      return false;
    }
    
    if (is_open()) {
      RCLCPP_DEBUG(logger_, "USB connection already open");
      return true;
    }
    
    try {
      // Initialize the USB connection first
      usbc_->init();
      
      // Now open the device
      bool result = usbc_->open_device();
      if (result) {
        RCLCPP_INFO(logger_, "USB connection opened successfully");
      } else {
        RCLCPP_ERROR(logger_, "Failed to open USB connection");
      }
      return result;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Error opening USB connection: %s", e.what());
      return false;
    }
  }

  void close() override {
    if (usbc_ && is_open()) {
      usbc_->shutdown();
      RCLCPP_INFO(logger_, "USB connection closed");
    }
  }

private:
  // Wait for an ACK/NAK response to a message
  bool wait_for_ack(uint8_t msg_class, uint8_t msg_id, int timeout_ms = 1000) {
    auto start_time = std::chrono::steady_clock::now();
    
    std::shared_ptr<Frame> frame;
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time).count() < timeout_ms)
    {
      // Try to read a message
      frame = std::make_shared<Frame>();
      ReadResult read_result = read(frame, 100);  // Short timeout for polling
      
      if (read_result.status == ReadStatus::SUCCESS) {
        // Check if it's an ACK or NAK message
        if (frame->msg_class == ACK_CLASS) {
          if (frame->msg_id == ACK_ID) {
            // ACK message - check if it's for our message
            if (frame->length >= 2 && frame->payload != nullptr) {
              uint8_t ack_class = frame->payload[0];
              uint8_t ack_id = frame->payload[1];
              
              if (ack_class == msg_class && ack_id == msg_id) {
                RCLCPP_DEBUG(logger_, "Received ACK for message class 0x%02X, ID 0x%02X", 
                          msg_class, msg_id);
                return true;
              }
            }
          } else if (frame->msg_id == NAK_ID) {
            // NAK message - check if it's for our message
            if (frame->length >= 2 && frame->payload != nullptr) {
              uint8_t nak_class = frame->payload[0];
              uint8_t nak_id = frame->payload[1];
              
              if (nak_class == msg_class && nak_id == msg_id) {
                RCLCPP_WARN(logger_, "Received NAK for message class 0x%02X, ID 0x%02X", 
                          msg_class, msg_id);
                return false;
              }
            }
          }
        }
      } else if (read_result.status == ReadStatus::TIMEOUT) {
        // Short timeout - continue polling
        continue;
      } else {
        // Error or no data
        RCLCPP_DEBUG(logger_, "Error or no data while waiting for ACK");
      }
    }
    
    RCLCPP_WARN(logger_, "Timeout waiting for ACK/NAK for message class 0x%02X, ID 0x%02X", 
              msg_class, msg_id);
    return false;  // Timeout
  }
  
  // Class constants
  static constexpr uint8_t ACK_CLASS = 0x05;
  static constexpr uint8_t ACK_ID = 0x01;
  static constexpr uint8_t NAK_ID = 0x00;
  
  // Member variables
  std::shared_ptr<usb::Connection> usbc_;
  rclcpp::Logger logger_;
};

std::shared_ptr<UbxTransceiver> UbxTransceiverFactory::create_usb_transceiver(
  std::shared_ptr<usb::Connection> usbc,
  rclcpp::Logger logger)
{
  return std::make_shared<RealUsbTransceiver>(usbc, logger);
}

std::shared_ptr<UbxTransceiver> UbxTransceiverFactory::create_usb_transceiver(
  rclcpp::Node * node,
  std::shared_ptr<usb::Connection> usbc)
{
  return std::make_shared<RealUsbTransceiver>(usbc, node->get_logger());
}

}  // namespace ubx
