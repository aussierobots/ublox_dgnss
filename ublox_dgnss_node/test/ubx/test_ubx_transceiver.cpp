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
 * @file test_ubx_transceiver.cpp
 * @brief Simple unit tests for the UbxTransceiver interface implementation
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>

#include "ublox_dgnss_node/ubx/ubx_transceiver.hpp"
#include "ublox_dgnss_node/ubx/mock_ubx_transceiver.hpp"

namespace ubx {
namespace test {

/**
 * @brief Test fixture for UbxTransceiver interface
 */
class UbxTransceiverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a mock transceiver
    mock_transceiver_ = std::make_shared<MockUbxTransceiver>();
  }

  std::shared_ptr<MockUbxTransceiver> mock_transceiver_;
};

/**
 * @brief Test that the mock transceiver can be instantiated
 */
TEST_F(UbxTransceiverTest, Instantiation)
{
  ASSERT_NE(mock_transceiver_, nullptr);
}

/**
 * @brief Test the write method of the transceiver
 */
TEST_F(UbxTransceiverTest, WriteMessage)
{
  // Create a simple frame to send
  auto frame = std::make_shared<ubx::Frame>();
  frame->msg_class = 0x01;
  frame->msg_id = 0x02;
  frame->length = 0;

  // Set up expectation
  EXPECT_CALL(*mock_transceiver_, write(testing::_))
    .Times(1)
    .WillOnce(testing::Return(WriteResult{AckNack::ACK}));

  // Write the frame
  WriteResult result = mock_transceiver_->write(frame);
  EXPECT_EQ(result.status, AckNack::ACK);
}

/**
 * @brief Test the read method of the transceiver
 */
TEST_F(UbxTransceiverTest, ReadMessage)
{
  // Set up a frame to be "read"
  auto expected_frame = std::make_shared<ubx::Frame>();
  expected_frame->msg_class = 0x01;
  expected_frame->msg_id = 0x02;
  expected_frame->length = 0;

  // Set up expectation for read call
  EXPECT_CALL(*mock_transceiver_, read(testing::_, testing::_))
    .Times(1)
    .WillOnce(testing::DoAll(
      testing::SetArgReferee<0>(expected_frame),
      testing::Return(ReadResult{ReadStatus::SUCCESS})));

  // Read a frame
  std::shared_ptr<ubx::Frame> received_frame;
  ReadResult result = mock_transceiver_->read(received_frame, 1000);
  
  // Verify the read result and frame
  EXPECT_EQ(result.status, ReadStatus::SUCCESS);
  ASSERT_NE(received_frame, nullptr);
  EXPECT_EQ(received_frame->msg_class, expected_frame->msg_class);
  EXPECT_EQ(received_frame->msg_id, expected_frame->msg_id);
}

/**
 * @brief Test the open and is_open methods
 */
TEST_F(UbxTransceiverTest, ConnectionManagement)
{
  // Test open
  EXPECT_CALL(*mock_transceiver_, open())
    .Times(1)
    .WillOnce(testing::Return(true));
  
  bool open_result = mock_transceiver_->open();
  EXPECT_TRUE(open_result);

  // Test is_open
  EXPECT_CALL(*mock_transceiver_, is_open())
    .Times(1)
    .WillOnce(testing::Return(true));
  
  bool is_open_result = mock_transceiver_->is_open();
  EXPECT_TRUE(is_open_result);

  // Test close
  EXPECT_CALL(*mock_transceiver_, close())
    .Times(1);
  
  mock_transceiver_->close();
}

}  // namespace test
}  // namespace ubx

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
