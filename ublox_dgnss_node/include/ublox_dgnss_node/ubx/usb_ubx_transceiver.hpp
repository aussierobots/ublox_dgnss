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
 * @file usb_ubx_transceiver.hpp
 * @brief Definition of the UsbUbxTransceiver class for USB-based UBX message communication
 */

#ifndef UBLOX_DGNSS_NODE__UBX__USB_UBX_TRANSCEIVER_HPP_
#define UBLOX_DGNSS_NODE__UBX__USB_UBX_TRANSCEIVER_HPP_

#include <memory>
#include <vector>
#include <chrono>
#include "ublox_dgnss_node/ubx/ubx_transceiver.hpp"
#include "ublox_dgnss_node/usb.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ubx
{

/**
 * @brief USB implementation of the UbxTransceiver interface
 *
 * This class implements the UbxTransceiver interface using a USB connection
 */
class UsbUbxTransceiver : public UbxTransceiver
{
public:
  /**
   * @brief Constructor with USB connection and logger
   * @param usbc USB connection
   * @param logger ROS logger
   */
  UsbUbxTransceiver(
    std::shared_ptr<usb::Connection> usbc,
    rclcpp::Logger logger);

  /**
   * @brief Write a UBX message to the device and get ACK/NAK status.
   * @param frame Shared pointer to the constant Frame object to be written.
   * @return WriteResult indicating status of the write and acknowledgement.
   */
  WriteResult write(std::shared_ptr<const ubx::Frame> frame) override;

  /**
   * @brief Read a UBX message from the device.
   * @param frame Shared pointer to a Frame object to be populated with the read message.
   * @param timeout_ms Timeout in milliseconds for the read operation.
   * @return ReadResult indicating status of the read operation.
   */
  ReadResult read(std::shared_ptr<ubx::Frame> & frame, int timeout_ms = 1000) override;

  /**
   * @brief Check if the transceiver is open/connected.
   * @return True if open/connected, false otherwise.
   */
  bool is_open() override;

  /**
   * @brief Open/connect to the device.
   * @return True if opening/connection was successful, false otherwise.
   */
  bool open() override;

  /**
   * @brief Close/disconnect from the device.
   */
  void close() override;

  /**
   * @brief Send a UBX message
   * @param frame The UBX frame to send
   * @return True if the message was sent successfully
   */
  bool send_ubx_message(const std::shared_ptr<ubx::Frame> & frame);

  /**
   * @brief Receive a UBX message
   * @param buffer Buffer to store the received message
   * @param timeout_ms Timeout in milliseconds
   * @return Number of bytes received, or -1 on error
   */
  int receive_ubx_message(std::vector<u1_t> & buffer, int timeout_ms = 1000);

  /**
   * @brief Wait for an ACK/NAK response to a message
   * @param msg_class Message class to wait for ACK
   * @param msg_id Message ID to wait for ACK
   * @param timeout_ms Timeout in milliseconds
   * @return True if ACK received, false if NAK or timeout
   */
  bool wait_for_ack(uint8_t msg_class, uint8_t msg_id, int timeout_ms = 1000);

  /**
   * @brief Check if the transceiver is connected
   * @return True if connected
   */
  bool is_connected();

  /**
   * @brief Connect to the device
   * @return True if connection successful
   */
  bool connect();

  /**
   * @brief Disconnect from the device
   */
  void disconnect();

private:
  std::shared_ptr<usb::Connection> usbc_;  ///< USB connection
  rclcpp::Logger logger_;                   ///< ROS logger
};

}  // namespace ubx

#endif  // UBLOX_DGNSS_NODE__UBX__USB_UBX_TRANSCEIVER_HPP_
