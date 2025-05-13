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
 * @file test_ubx_cfg_handler_new.cpp
 * @brief Unit tests for the UbxCfgHandler class using the UbxTransceiver interface
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <vector>

#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_handler.hpp"
#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_item.hpp"
#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_parameter.hpp"
#include "ublox_dgnss_node/ubx/mock_ubx_transceiver.hpp"

// Test fixture for UbxCfgHandler using GMock
class UbxCfgHandlerTest : public ::testing::Test
{
protected:
  std::shared_ptr<MockUbxTransceiver> mock_transceiver_;
  std::shared_ptr<ubx::cfg::Context> context_;
  std::shared_ptr<ubx::cfg::UbxCfgHandler> handler_;

  void SetUp() override
  {
    mock_transceiver_ = std::make_shared<MockUbxTransceiver>();
    context_ = std::make_shared<ubx::cfg::Context>();
    ubx::cfg::ubx_cfg_item_t item1{"CFG-RATE-MEAS", 0x30210001, ubx::U2, 1.0, ubx::cfg::NA};
    ubx::cfg::ubx_cfg_item_t item2{"CFG-TP-PULSE_LENGTH_DEF", 0x30050031, ubx::U4, 1.0, ubx::cfg::NA};
    context_->add_parameter("CFG-RATE-MEAS", item1);
    context_->add_parameter("CFG-TP-PULSE_LENGTH_DEF", item2);

    handler_ = std::make_shared<ubx::cfg::UbxCfgHandler>(mock_transceiver_, context_);
  }
};

// Test fixture for UbxCfgParameter
class UbxCfgParameterTest : public ::testing::Test
{
protected:
  ubx::cfg::UbxCfgParameter parameter_;
  ubx::cfg::ubx_cfg_item_t item_;

  UbxCfgParameterTest() : item_{"CFG-UART1-BAUDRATE", 0x20110001, ubx::U4, 1.0, ubx::cfg::NA}, parameter_("CFG-UART1-BAUDRATE", item_) {}
};

// Test that the UbxCfgHandler initializes correctly
TEST_F(UbxCfgHandlerTest, Initialization)
{
  ASSERT_NE(handler_, nullptr);
  ASSERT_EQ(handler_->get_transceiver(), mock_transceiver_);
  ASSERT_EQ(handler_->get_context(), context_);
}

// Test polling a single known parameter
TEST_F(UbxCfgHandlerTest, PollSingleParameter)
{
  std::string param_name = "CFG-RATE-MEAS";
  ubx::cfg::key_id_t key_id = 0x30210001;

  EXPECT_CALL(
    *mock_transceiver_,
    send(testing::Truly(
      [key_id](const ubx::frame_ptr & frame) {
        if (frame->msg_class == ubx::cfg::valget::UbxCfgValGetPayload::MSG_CLASS &&
          frame->msg_id == ubx::cfg::valget::UbxCfgValGetPayload::MSG_ID) {
          auto payload = ubx::cfg::valget::UbxCfgValGetPayload::from_frame(frame);
          return payload.version == 0 && payload.layer == 0 && payload.keys.size() == 1 &&
                 payload.keys[0] == key_id;
        }
        return false;
      }))) 
  .Times(1);

  handler_->poll_parameter(param_name);
}

// Test polling multiple parameters
TEST_F(UbxCfgHandlerTest, PollMultipleParameters)
{
  std::vector<std::string> param_names = {"CFG-RATE-MEAS", "CFG-TP-PULSE_LENGTH_DEF"};
  std::vector<ubx::cfg::key_id_t> key_ids = {0x30210001, 0x30050031};

  EXPECT_CALL(
    *mock_transceiver_,
    send(testing::Truly(
      [&key_ids](const ubx::frame_ptr & frame) {
        if (frame->msg_class == ubx::cfg::valget::UbxCfgValGetPayload::MSG_CLASS &&
          frame->msg_id == ubx::cfg::valget::UbxCfgValGetPayload::MSG_ID) {
          auto payload = ubx::cfg::valget::UbxCfgValGetPayload::from_frame(frame);
          if (payload.version == 0 && payload.layer == 0 && payload.keys.size() == key_ids.size()) {
            for (size_t i = 0; i < key_ids.size(); ++i) {
              if (payload.keys[i] != key_ids[i]) {
                return false;
              }
            }
            return true;
          }
        }
        return false;
      }))) 
  .Times(1);

  handler_->poll_parameters(param_names);
}

// Test setting a single parameter (synchronously)
TEST_F(UbxCfgHandlerTest, SetSingleParameterSync)
{
  std::string param_name = "CFG-RATE-MEAS";
  uint16_t new_value = 500;  
  ubx::cfg::key_id_t key_id = 0x30210001;

  ubx::cfg::valset::UbxCfgValSetPayload expected_payload;
  expected_payload.version = 0x01;  
  expected_payload.ram = true;
  expected_payload.bbr = true;
  expected_payload.flash = false;
  expected_payload.items.push_back({key_id, new_value});
  auto expected_frame = expected_payload.to_frame();

  EXPECT_CALL(
    *mock_transceiver_,
    send(testing::Truly(
      [&expected_frame](const ubx::frame_ptr & frame) {
        if (frame->msg_class != ubx::cfg::valset::UbxCfgValSetPayload::MSG_CLASS ||
            frame->msg_id != ubx::cfg::valset::UbxCfgValSetPayload::MSG_ID) {
          return false;
        }
        return frame->to_vector() == expected_frame->to_vector();
      }))) 
  .Times(1);

  ubx::cfg::ack::UbxCfgAckPayload ack_payload;
  ack_payload.msg_class = ubx::cfg::valset::UbxCfgValSetPayload::MSG_CLASS;
  ack_payload.msg_id = ubx::cfg::valset::UbxCfgValSetPayload::MSG_ID;
  auto ack_frame = ack_payload.to_frame(ubx::cfg::ack::UbxCfgAckPayload::MSG_ID_ACK); 

  EXPECT_CALL(*mock_transceiver_, receive(testing::_, testing::_))
    .WillOnce(testing::Return(ack_frame));

  bool success = handler_->set_parameter_sync(param_name, new_value);
  ASSERT_TRUE(success);

  auto updated_param = context_->get_parameter(param_name);
  ASSERT_NE(updated_param, nullptr);
  EXPECT_EQ(updated_param->get_value<uint16_t>(), new_value);
  EXPECT_FALSE(updated_param->is_modified()); 
}

// Test setting a parameter that doesn't exist in context
TEST_F(UbxCfgHandlerTest, SetNonExistentParameter)
{
  std::string param_name = "CFG-NON-EXISTENT";
  uint16_t value = 123;

  EXPECT_CALL(*mock_transceiver_, send(testing::_)).Times(0);
  EXPECT_CALL(*mock_transceiver_, receive(testing::_, testing::_)).Times(0);

  bool success = handler_->set_parameter_sync(param_name, value);
  ASSERT_FALSE(success);
}

// Test for MON-VER polling
TEST_F(UbxCfgHandlerTest, PollMonVer)
{
  EXPECT_CALL(
    *mock_transceiver_,
    send(testing::Truly(
      [](const ubx::frame_ptr & frame) {
        return frame->msg_class == ubx::mon::ver::MonVerPollPayload::MSG_CLASS &&
               frame->msg_id == ubx::mon::ver::MonVerPollPayload::MSG_ID &&
               frame->length == 0;
      }))) 
  .Times(1);
  handler_->poll_mon_ver();
}

// TODO(ahcorde): Test receiving MON-VER response
// TEST_F(UbxCfgHandlerTest, ReceiveMonVerResponse)
// {
//   // Simulate receiving MON-VER data
//   ubx::mon::ver::MonVerPayload mon_ver_payload;
//   // ... populate mon_ver_payload ...
//   auto mon_ver_frame = mon_ver_payload.to_frame();

//   // Trigger the receive logic (e.g. via a public method or internal call if refactored for testability)
//   // For now, let's assume there's a method that internally calls receive and processes
//   // This might require adding a test-specific method or refactoring the handler's receive loop

//   // Assertions on how the handler processes the MON-VER response
// }

// TODO(ahcorde): Test polling of CFG-NAVX5
TEST_F(UbxCfgHandlerTest, PollCfgNavRate)
{
  // Test polling of CFG-NAVX5-RATE
}

// TODO(ahcorde): Test polling CFG-NAVX5 parameters

// TODO(ahcorde): Test setting CFG-NAVX5 parameters

// TODO(ahcorde): Add more tests

// Test the context management for the UbxCfgHandler
TEST_F(UbxCfgHandlerTest, ContextManagement)
{
  auto handler = std::make_shared<ubx::UbxCfgHandler<MockUbxTransceiver>>();

  ASSERT_NE(handler->get_context(), nullptr);

  auto new_context = std::make_shared<ubx::cfg::Context>();
  handler->set_context(new_context);

  ASSERT_EQ(handler->get_context(), new_context);
}

TEST_F(UbxCfgParameterTest, BasicOperation)
{
  ubx::cfg::ubx_cfg_item_t item1{"CFG-HW-ANT_CFG_VOLTCTRL", 0x10240011, ubx::I1, 1.0, ubx::cfg::NA};
  ubx::cfg::UbxCfgParameter param1("CFG-HW-ANT_CFG_VOLTCTRL", item1);
  param1.set_value(static_cast<int8_t>(20));
  EXPECT_EQ(param1.get_value<int8_t>(), 20);
  
  ubx::cfg::ubx_cfg_item_t item2{"CFG-HW-ANT_SUP_SWITCH_PIN", 0x10240005, ubx::U1, 1.0, ubx::cfg::NA};
  ubx::cfg::UbxCfgParameter param2("CFG-HW-ANT_SUP_SWITCH_PIN", item2);
  EXPECT_EQ(param2.get_value<uint8_t>(), 10);
}

TEST_F(UbxCfgParameterTest, BehaviorChanges)
{
  ubx::cfg::ubx_cfg_item_t item{"CFG-UART1-BAUDRATE", 0x20110001, ubx::U4, 1.0, ubx::cfg::NA};
  ubx::cfg::UbxCfgParameter param("CFG-UART1-BAUDRATE", item);

  param.set_value(static_cast<uint32_t>(115200));
  EXPECT_EQ(param.get_value<uint32_t>(), 115200);
  EXPECT_TRUE(param.is_modified());

  param.reset_modified();
  EXPECT_FALSE(param.is_modified());

  param.set_value(static_cast<uint32_t>(9600));
  EXPECT_EQ(param.get_value<uint32_t>(), 9600);
  EXPECT_TRUE(param.is_modified());
}

TEST_F(UbxCfgParameterTest, ValueConversion)
{
  ubx::cfg::ubx_cfg_item_t item_u4{"CFG-UART1-BAUDRATE", 0x20110001, ubx::U4, 1.0, ubx::cfg::NA};
  ubx::cfg::UbxCfgParameter param_u4("CFG-UART1-BAUDRATE", item_u4);

  std::string val_str_u4 = "115200";
  ubx::value_t ubx_val_u4 = param_u4.string_to_ubx_value(val_str_u4);
  EXPECT_EQ(ubx_val_u4.u4, 115200U);
  EXPECT_EQ(param_u4.ubx_value_to_string(ubx_val_u4), val_str_u4);

  ubx::cfg::ubx_cfg_item_t item_i1{"CFG-NAVSPG-ACKAIDING", 0x20110066, ubx::I1, 1.0, ubx::cfg::NA};
  ubx::cfg::UbxCfgParameter param_i1("CFG-NAVSPG-ACKAIDING", item_i1);
  std::string val_str_i1 = "-5";
  ubx::value_t ubx_val_i1 = param_i1.string_to_ubx_value(val_str_i1);
  EXPECT_EQ(ubx_val_i1.i1, -5);
  EXPECT_EQ(param_i1.ubx_value_to_string(ubx_val_i1), val_str_i1);

  ubx::cfg::ubx_cfg_item_t item_l{"CFG-INFMSG-UBX_USB", 0x20920003, ubx::L, 1.0, ubx::cfg::NA};
  ubx::cfg::UbxCfgParameter param_l("CFG-INFMSG-UBX_USB", item_l);
  std::string val_str_l_true = "true";
  ubx::value_t ubx_val_l_true = param_l.string_to_ubx_value(val_str_l_true);
  EXPECT_EQ(ubx_val_l_true.l, true);
  EXPECT_EQ(param_l.ubx_value_to_string(ubx_val_l_true), "1");  

  std::string val_str_l_false = "false";
  ubx::value_t ubx_val_l_false = param_l.string_to_ubx_value(val_str_l_false);
  EXPECT_EQ(ubx_val_l_false.l, false);
  EXPECT_EQ(param_l.ubx_value_to_string(ubx_val_l_false), "0");

  std::string val_str_x1 = "0x01";  
  ubx::value_t ubx_val_x1 = param_l.string_to_ubx_value(val_str_x1);  
  EXPECT_EQ(ubx_val_x1.x1, 0x01);
  EXPECT_EQ(param_l.ubx_value_to_string(ubx_val_x1), "1");  
}

}  // namespace ubx
