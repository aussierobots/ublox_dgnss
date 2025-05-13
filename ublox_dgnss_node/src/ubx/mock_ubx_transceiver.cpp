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
 * @file mock_ubx_transceiver.cpp
 * @brief Implementation of the MockUbxTransceiver class for testing
 */

#include "ublox_dgnss_node/ubx/mock_ubx_transceiver.hpp"
#include "ublox_dgnss_node/ubx/ubx_ack.hpp"

namespace ubx
{

MockUbxTransceiver::MockUbxTransceiver()
: last_sent_message_(nullptr)
{
  // Set default behavior for the mock methods
  using ::testing::_;
  using ::testing::Return;
  using ::testing::Invoke;

  // Default behavior for send_ubx_message
  ON_CALL(*this, send_ubx_message(_))
  .WillByDefault(Invoke([this](const std::shared_ptr<ubx::Frame> & frame) {
      last_sent_message_ = frame;

      // Check if we have a handler for this message
      auto key = std::make_pair(frame->msg_class, frame->msg_id);
      if (handlers_.find(key) != handlers_.end()) {
        // Call the handler to generate a response
        auto response = handlers_[key](frame);

        // Store the response for later retrieval
        responses_[key] = response;
      }

      return true;
    }));

  // Default behavior for receive_ubx_message
  ON_CALL(*this, receive_ubx_message(_, _))
  .WillByDefault(Invoke([this](std::vector<u1_t> & buffer, int /*timeout_ms*/) {
      if (!last_sent_message_) {
        return -1;  // No message sent yet
      }

      auto key = std::make_pair(last_sent_message_->msg_class, last_sent_message_->msg_id);
      if (responses_.find(key) != responses_.end()) {
        // Copy the response to the buffer
        auto & response = responses_[key];
        if (buffer.size() < response.size()) {
          buffer.resize(response.size());
        }

        std::copy(response.begin(), response.end(), buffer.begin());
        return static_cast<int>(response.size());
      }

      return 0;  // No response available
    }));

  // Default behavior for wait_for_ack
  ON_CALL(*this, wait_for_ack(_, _, _))
  .WillByDefault(Return(true));

  // Default behavior for is_connected
  ON_CALL(*this, is_connected())
  .WillByDefault(Return(true));

  // Default behavior for connect
  ON_CALL(*this, connect())
  .WillByDefault(Return(true));

  // Default behavior for disconnect
  ON_CALL(*this, disconnect())
  .WillByDefault([]() {});
}

void MockUbxTransceiver::set_response(
  uint8_t msg_class, uint8_t msg_id,
  const std::vector<u1_t> & response)
{
  responses_[std::make_pair(msg_class, msg_id)] = response;
}

void MockUbxTransceiver::set_handler(
  uint8_t msg_class, uint8_t msg_id,
  std::function<std::vector<u1_t>(const std::shared_ptr<ubx::Frame> &)> handler)
{
  handlers_[std::make_pair(msg_class, msg_id)] = handler;
}

std::shared_ptr<ubx::Frame> MockUbxTransceiver::get_last_sent_message() const
{
  return last_sent_message_;
}

}  // namespace ubx
