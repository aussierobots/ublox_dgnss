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
 * @file mock_ubx_transceiver.hpp
 * @brief Definition of the MockUbxTransceiver class for testing
 */

#ifndef UBLOX_DGNSS_NODE__UBX__MOCK_UBX_TRANSCEIVER_HPP_
#define UBLOX_DGNSS_NODE__UBX__MOCK_UBX_TRANSCEIVER_HPP_

#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <map>
#include <functional>
#include <gmock/gmock.h>
#include "ublox_dgnss_node/ubx/ubx_transceiver.hpp"

namespace ubx
{

/**
 * @brief Mock implementation of the UbxTransceiver interface for testing
 */
class MockUbxTransceiver : public UbxTransceiver
{
public:
  /**
   * @brief Constructor
   */
  MockUbxTransceiver();

  /**
   * @brief Destructor
   */
  ~MockUbxTransceiver() override = default;

  // Mock methods that match the UbxTransceiver interface
  MOCK_METHOD(WriteResult, write, (std::shared_ptr<const ubx::Frame> frame), (override));
  MOCK_METHOD(ReadResult, read, (std::shared_ptr<ubx::Frame> & frame, int timeout_ms), (override));
  MOCK_METHOD(bool, is_open, (), (override));
  MOCK_METHOD(bool, open, (), (override));
  MOCK_METHOD(void, close, (), (override));

  // Legacy mock methods for backward compatibility
  MOCK_METHOD(bool, send_ubx_message, (const std::shared_ptr<ubx::Frame> & frame), ());
  MOCK_METHOD(int, receive_ubx_message, (std::vector<u1_t> & buffer, int timeout_ms), ());
  MOCK_METHOD(bool, wait_for_ack, (uint8_t msg_class, uint8_t msg_id, int timeout_ms), ());
  MOCK_METHOD(bool, is_connected, (), ());
  MOCK_METHOD(bool, connect, (), ());
  MOCK_METHOD(void, disconnect, (), ());

  /**
   * @brief Set a custom response for a specific message
   * @param msg_class Message class
   * @param msg_id Message ID
   * @param response Response buffer
   */
  void set_response(uint8_t msg_class, uint8_t msg_id, const std::vector<u1_t> & response);

  /**
   * @brief Set a custom handler for a specific message
   * @param msg_class Message class
   * @param msg_id Message ID
   * @param handler Handler function
   */
  void set_handler(
    uint8_t msg_class, uint8_t msg_id,
    std::function<std::vector<u1_t>(const std::shared_ptr<ubx::Frame> &)> handler);

  /**
   * @brief Get the last sent message
   * @return Last sent message
   */
  std::shared_ptr<ubx::Frame> get_last_sent_message() const;

private:
  std::shared_ptr<ubx::Frame> last_sent_message_;
  std::map<std::pair<uint8_t, uint8_t>, std::vector<u1_t>> responses_;
  std::map<std::pair<uint8_t, uint8_t>,
    std::function<std::vector<u1_t>(const std::shared_ptr<ubx::Frame> &)>> handlers_;
};

}  // namespace ubx

#endif  // UBLOX_DGNSS_NODE__UBX__MOCK_UBX_TRANSCEIVER_HPP_
