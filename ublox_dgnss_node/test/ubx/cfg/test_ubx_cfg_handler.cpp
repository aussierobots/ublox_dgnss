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
 * @file test_ubx_cfg_handler.cpp
 * @brief Unit tests for the UbxCfgHandler class
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <filesystem>

#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_handler.hpp"
#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_parameter_loader.hpp"
#include "ublox_dgnss_node/ubx/ubx_types.hpp"
#include "ublox_dgnss_node/ubx/ubx.hpp"

// Define a simple UbxMessage class for testing
namespace ubx {
  struct UbxMessage {
    msg_class_t msg_class;
    msg_id_t msg_id;
    std::vector<u1_t> payload;
  };
}

// Mock classes
namespace ubx::cfg
{

class MockUbxTransceiver
{
public:
  MOCK_METHOD(void, send_ubx_message, (const ubx::UbxMessage & msg));
  MOCK_METHOD(bool, receive_ubx_message, (ubx::UbxMessage & msg));
};

class MockParameterLoader
{
public:
  MOCK_METHOD(bool, load, ());
  MOCK_METHOD(std::optional<UbxCfgParameter>, get_parameter_by_name, (const std::string & name));
  MOCK_METHOD(std::optional<UbxCfgParameter>, get_parameter_by_key_id, (const ubx_key_id_t & key_id));
  MOCK_METHOD(std::vector<UbxCfgParameter>, get_all_parameters, ());
  MOCK_METHOD(std::vector<UbxCfgParameter>, filter_parameters_by_device_type, (const std::string & device_type));
};

// Extend UbxCfgHandler for testing
class TestUbxCfgHandler : public UbxCfgHandler
{
public:
  TestUbxCfgHandler(
    rclcpp::Node * node,
    std::shared_ptr<MockUbxTransceiver> transceiver,
    std::shared_ptr<MockParameterLoader> loader,
    const std::string & device_type)
    : UbxCfgHandler(node, nullptr, "", device_type),
      transceiver_(transceiver),
      loader_(loader)
  {
  }

  // Expose protected methods for testing
  std::string get_detected_firmware_version() const
  {
    return detected_firmware_version_;
  }

private:
  std::shared_ptr<MockUbxTransceiver> transceiver_;
  std::shared_ptr<MockParameterLoader> loader_;
};

}  // namespace ubx::cfg

namespace
{

using ::testing::Return;
using ::testing::_;
using ::testing::AtLeast;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Invoke;

class UbxCfgHandlerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create mock objects
    mock_transceiver_ = std::make_shared<ubx::cfg::MockUbxTransceiver>();
    mock_loader_ = std::make_shared<ubx::cfg::MockParameterLoader>();
    
    // Create handler
    handler_ = std::make_unique<ubx::cfg::TestUbxCfgHandler>(
      nullptr,
      mock_transceiver_,
      mock_loader_,
      "ZED-F9P"
    );
    
    // Create test parameters
    createTestParameters();
    
    // Set up mock behavior
    ON_CALL(*mock_loader_, load()).WillByDefault(Return(true));
    ON_CALL(*mock_loader_, get_all_parameters()).WillByDefault(Return(test_parameters_));
    ON_CALL(*mock_loader_, get_available_device_types()).WillByDefault(Return(std::vector<std::string>{"ZED-F9P", "ZED-F9R"}));
    ON_CALL(*mock_loader_, get_available_firmware_versions(_)).WillByDefault(Return(std::vector<std::string>{"HPG 1.13", "HPG 1.30", "HPG 1.32"}));
    
    // Set up parameter lookup
    ON_CALL(*mock_loader_, get_parameter_by_name(_))
      .WillByDefault([this](const std::string & name) {
        if (param_map_by_name_.find(name) != param_map_by_name_.end()) {
          return std::optional<ubx::cfg::UbxCfgParameter>(param_map_by_name_[name]);
        }
        return std::optional<ubx::cfg::UbxCfgParameter>();
      });
    
    ON_CALL(*mock_loader_, get_parameter_by_key_id(_))
      .WillByDefault([this](const ubx::cfg::ubx_key_id_t & key_id) {
        if (param_map_by_key_id_.find(key_id.all) != param_map_by_key_id_.end()) {
          return std::optional<ubx::cfg::UbxCfgParameter>(param_map_by_key_id_[key_id.all]);
        }
        return std::optional<ubx::cfg::UbxCfgParameter>();
      });
    
    // Set up transceiver behavior for firmware version detection
    ON_CALL(*mock_transceiver_, send_ubx_message(_))
      .WillByDefault(Return());
    
    // Set up transceiver behavior for firmware version detection
    ON_CALL(*mock_transceiver_, receive_ubx_message(_))
      .WillByDefault(Return(true));
  }

  void createTestParameters()
  {
    // Create test parameters
    
    // UART1 Baudrate parameter
    std::vector<std::string> applicable_devices = {"ZED-F9P", "ZED-F9R"};
    
    std::map<std::string, ubx::cfg::FirmwareSupport> firmware_support;
    
    // Add firmware support for ZED-F9P
    ubx::cfg::FirmwareSupport f9p_support;
    f9p_support.since = "HPG 1.13";
    firmware_support["ZED-F9P"] = f9p_support;
    
    // Add firmware support for ZED-F9R
    ubx::cfg::FirmwareSupport f9r_support;
    f9r_support.since = "HPS 1.13";
    firmware_support["ZED-F9R"] = f9r_support;
    
    ubx::cfg::ubx_key_id_t uart_key_id;
    uart_key_id.all = 0x40520001;
    
    ubx::cfg::UbxCfgParameter uart_param(
      "CFG_UART1_BAUDRATE",
      uart_key_id,
      ubx::ubx_type_t::U4,
      1.0,
      ubx::cfg::ubx_unit_t::NA,
      applicable_devices,
      "UART1 baud rate",
      "UART",
      firmware_support,
      {},  // No possible values
      "38400",
      "9600",
      "921600"
    );
    
    // Measurement rate parameter
    std::map<std::string, ubx::cfg::FirmwareSupport> rate_firmware_support;
    
    // Add firmware support for ZED-F9P
    ubx::cfg::FirmwareSupport rate_f9p_support;
    rate_f9p_support.since = "HPG 1.13";
    
    // Add behavior change
    ubx::cfg::BehaviorChange change;
    change.version = "HPG 1.32";
    change.description = "Improved handling and performance";
    rate_f9p_support.behavior_changes.push_back(change);
    
    rate_firmware_support["ZED-F9P"] = rate_f9p_support;
    
    // Add firmware support for ZED-F9R
    ubx::cfg::FirmwareSupport rate_f9r_support;
    rate_f9r_support.since = "HPS 1.13";
    rate_firmware_support["ZED-F9R"] = rate_f9r_support;
    
    ubx::cfg::ubx_key_id_t rate_key_id;
    rate_key_id.all = 0x30210001;
    
    ubx::cfg::UbxCfgParameter rate_param(
      "CFG_RATE_MEAS",
      rate_key_id,
      ubx::ubx_type_t::U2,
      0.001,
      ubx::cfg::ubx_unit_t::S,
      applicable_devices,
      "Measurement rate",
      "RATE",
      rate_firmware_support,
      {},  // No possible values
      "1000",
      "50",
      "10000"
    );
    
    // Dynamic model parameter
    std::map<std::string, ubx::cfg::FirmwareSupport> dyn_firmware_support;
    
    // Add firmware support for ZED-F9P
    ubx::cfg::FirmwareSupport dyn_f9p_support;
    dyn_f9p_support.since = "HPG 1.30";
    dyn_firmware_support["ZED-F9P"] = dyn_f9p_support;
    
    // Add firmware support for ZED-F9R
    ubx::cfg::FirmwareSupport dyn_f9r_support;
    dyn_f9r_support.since = "HPS 1.20";
    dyn_firmware_support["ZED-F9R"] = dyn_f9r_support;
    
    // Create possible values for enum type
    std::map<std::string, std::string> possible_values;
    possible_values["DYN_MODEL_PORT"] = "0x00";
    possible_values["DYN_MODEL_STATIONARY"] = "0x02";
    possible_values["DYN_MODEL_PEDESTRIAN"] = "0x03";
    possible_values["DYN_MODEL_AUTOMOTIVE"] = "0x04";
    possible_values["DYN_MODEL_SEA"] = "0x05";
    possible_values["DYN_MODEL_AIRBORNE_1G"] = "0x06";
    possible_values["DYN_MODEL_AIRBORNE_2G"] = "0x07";
    possible_values["DYN_MODEL_AIRBORNE_4G"] = "0x08";
    possible_values["DYN_MODEL_WRIST"] = "0x09";
    
    ubx::cfg::ubx_key_id_t dyn_key_id;
    dyn_key_id.all = 0x20110021;
    
    ubx::cfg::UbxCfgParameter dyn_param(
      "CFG_NAVSPG_DYNMODEL",
      dyn_key_id,
      ubx::ubx_type_t::E1,
      1.0,
      ubx::cfg::ubx_unit_t::NA,
      applicable_devices,
      "Dynamic platform model",
      "NAVSPG",
      dyn_firmware_support,
      possible_values,
      "0x04",  // Automotive
      "0x00",
      "0x09"
    );
    
    // Add parameters to the test parameters vector
    test_parameters_.push_back(uart_param);
    test_parameters_.push_back(rate_param);
    test_parameters_.push_back(dyn_param);
    
    // Create parameter map for lookup
    for (const auto & param : test_parameters_) {
      param_map_by_name_[param.get_name()] = param;
      param_map_by_key_id_[param.get_key_id()] = param;
    }
  }
  
  std::optional<ubx::cfg::UbxCfgParameter> getMockParameterByName(const std::string & name)
  {
    if (param_map_by_name_.find(name) != param_map_by_name_.end()) {
      return std::optional<ubx::cfg::UbxCfgParameter>(param_map_by_name_[name]);
    }
    return std::optional<ubx::cfg::UbxCfgParameter>();
  }
  
  std::optional<ubx::cfg::UbxCfgParameter> getMockParameterByKeyId(const ubx::cfg::ubx_key_id_t & key_id)
  {
    if (param_map_by_key_id_.find(key_id.all) != param_map_by_key_id_.end()) {
      return std::optional<ubx::cfg::UbxCfgParameter>(param_map_by_key_id_[key_id.all]);
    }
    return std::optional<ubx::cfg::UbxCfgParameter>();
  }

  std::shared_ptr<rclcpp::Node> mock_node_;
  std::shared_ptr<ubx::cfg::MockUbxTransceiver> mock_transceiver_;
  std::shared_ptr<ubx::cfg::MockParameterLoader> mock_loader_;
  std::unique_ptr<ubx::cfg::TestUbxCfgHandler> handler_;
  
  std::vector<ubx::cfg::UbxCfgParameter> test_parameters_;
  std::map<std::string, ubx::cfg::UbxCfgParameter> param_map_by_name_;
  std::map<uint32_t, ubx::cfg::UbxCfgParameter> param_map_by_key_id_;
};

// Test initialization
TEST_F(UbxCfgHandlerTest, Initialize)
{
  // Set expectations
  EXPECT_CALL(*mock_loader_, load()).Times(1);
  EXPECT_CALL(*mock_transceiver_, send_ubx_message(_)).Times(AtLeast(1));
  
  // Initialize the handler
  EXPECT_TRUE(handler_->initialize());
  
  // Check that parameters were registered
  auto parameters = mock_node_->get_parameters({"CFG_UART1_BAUDRATE", "CFG_RATE_MEAS", "CFG_NAVSPG_DYNMODEL"});
  EXPECT_EQ(parameters.size(), 3u);
}

// Test parameter change handling
TEST_F(UbxCfgHandlerTest, ParameterChange)
{
  // Initialize the handler
  EXPECT_TRUE(handler_->initialize());
  
  // Set expectations for parameter change
  EXPECT_CALL(*mock_transceiver_, send_ubx_message(_)).Times(AtLeast(1));
  EXPECT_CALL(*mock_transceiver_, wait_for_ack(_, _, _)).WillRepeatedly(Return(true));
  
  // Create a parameter change
  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("CFG_UART1_BAUDRATE", 115200));
  
  // Handle parameter change
  auto result = handler_->on_parameter_change(parameters);
  
  // Check result
  EXPECT_TRUE(result.successful);
}

// Test parameter validation
TEST_F(UbxCfgHandlerTest, ParameterValidation)
{
  // Initialize the handler
  EXPECT_TRUE(handler_->initialize());
  
  // Test valid parameter value
  std::vector<rclcpp::Parameter> valid_params;
  valid_params.push_back(rclcpp::Parameter("CFG_UART1_BAUDRATE", 115200));
  auto valid_result = handler_->on_parameter_change(valid_params);
  EXPECT_TRUE(valid_result.successful);
  
  // Test invalid parameter value (below min)
  std::vector<rclcpp::Parameter> invalid_min_params;
  invalid_min_params.push_back(rclcpp::Parameter("CFG_UART1_BAUDRATE", 1200));
  auto invalid_min_result = handler_->on_parameter_change(invalid_min_params);
  EXPECT_FALSE(invalid_min_result.successful);
  
  // Test invalid parameter value (above max)
  std::vector<rclcpp::Parameter> invalid_max_params;
  invalid_max_params.push_back(rclcpp::Parameter("CFG_UART1_BAUDRATE", 1000000));
  auto invalid_max_result = handler_->on_parameter_change(invalid_max_params);
  EXPECT_FALSE(invalid_max_result.successful);
  
  // Test non-existent parameter
  std::vector<rclcpp::Parameter> non_existent_params;
  non_existent_params.push_back(rclcpp::Parameter("NON_EXISTENT", 42));
  auto non_existent_result = handler_->on_parameter_change(non_existent_params);
  EXPECT_FALSE(non_existent_result.successful);
}

// Test firmware version detection
TEST_F(UbxCfgHandlerTest, FirmwareVersionDetection)
{
  // Set up transceiver to return a firmware version
  ON_CALL(*mock_transceiver_, send_ubx_message(_))
    .WillByDefault([this](const ubx::UbxMessage & msg) {
      // Simulate receiving a UBX-MON-VER message response
      if (msg.msg_class == 0x0A && msg.msg_id == 0x04) {
        // Create a response message with firmware version
        std::string sw_version = "EXT CORE 1.00 (12345678)";
        std::string hw_version = "00080000";
        std::vector<std::string> extensions = {"FWVER=HPG 1.32"};
        
        // Call the handler's process_mon_ver method
        handler_->process_mon_ver(sw_version, hw_version, extensions);
      }
      return true;
    });
  
  // Initialize the handler
  EXPECT_TRUE(handler_->initialize());
  
  // Check that the firmware version was detected
  EXPECT_EQ(handler_->get_detected_firmware_version(), "HPG 1.32");
}

// Test parameter filtering by firmware version
TEST_F(UbxCfgHandlerTest, ParameterFilteringByFirmware)
{
  // Set up transceiver to return a firmware version
  ON_CALL(*mock_transceiver_, send_ubx_message(_))
    .WillByDefault([this](const ubx::UbxMessage & msg) {
      // Simulate receiving a UBX-MON-VER message response
      if (msg.msg_class == 0x0A && msg.msg_id == 0x04) {
        // Create a response message with firmware version
        std::string sw_version = "EXT CORE 1.00 (12345678)";
        std::string hw_version = "00080000";
        std::vector<std::string> extensions = {"FWVER=HPG 1.13"};  // Early firmware
        
        // Call the handler's process_mon_ver method
        handler_->process_mon_ver(sw_version, hw_version, extensions);
      }
      return true;
    });
  
  // Set up parameter loader to filter by firmware
  ON_CALL(*mock_loader_, filter_parameters_by_device_type("ZED-F9P"))
    .WillByDefault([this](const std::string & device_type) {
      // Return only parameters supported in HPG 1.13
      std::vector<ubx::cfg::UbxCfgParameter> filtered;
      filtered.push_back(test_parameters_[0]);  // UART
      filtered.push_back(test_parameters_[1]);  // RATE
      return filtered;
    });
  
  // Initialize the handler
  EXPECT_TRUE(handler_->initialize());
  
  // Check that only supported parameters were registered
  auto parameters = mock_node_->get_parameters({"CFG_UART1_BAUDRATE", "CFG_RATE_MEAS", "CFG_NAVSPG_DYNMODEL"});
  EXPECT_EQ(parameters.size(), 3u);  // All parameters are declared but some may be unavailable
  
  // Check that the dynamic model parameter is unavailable (not supported in HPG 1.13)
  auto dyn_param = mock_node_->get_parameter("CFG_NAVSPG_DYNMODEL");
  EXPECT_FALSE(dyn_param.get_parameter_value().get<int>());  // Should be default/unavailable
}

// Test parameter behavior changes
TEST_F(UbxCfgHandlerTest, ParameterBehaviorChanges)
{
  // Set up transceiver to return a firmware version
  ON_CALL(*mock_transceiver_, send_ubx_message(_))
    .WillByDefault([this](const ubx::UbxMessage & msg) {
      // Simulate receiving a UBX-MON-VER message response
      if (msg.msg_class == 0x0A && msg.msg_id == 0x04) {
        // Create a response message with firmware version
        std::string sw_version = "EXT CORE 1.00 (12345678)";
        std::string hw_version = "00080000";
        std::vector<std::string> extensions = {"FWVER=HPG 1.32"};  // Latest firmware with behavior changes
        
        // Call the handler's process_mon_ver method
        handler_->process_mon_ver(sw_version, hw_version, extensions);
      }
      return true;
    });
  
  // Initialize the handler
  EXPECT_TRUE(handler_->initialize());
  
  // Check that the behavior change was detected
  auto behavior_change = handler_->get_parameter_behavior_change("CFG_RATE_MEAS");
  EXPECT_EQ(behavior_change, "Improved handling and performance");
  
  // Check behavior change for parameter without changes
  auto no_change = handler_->get_parameter_behavior_change("CFG_UART1_BAUDRATE");
  EXPECT_TRUE(no_change.empty());
}

}  // namespace

// Run all tests
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
