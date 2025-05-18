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
 *
 * @brief Test file for the UbxCfgHandler class using MockUbxTransceiver
 * 
 * This test suite validates that the UbxCfgHandler correctly:
 * - Initializes with UbxTransceiver interface and parameter loader
 * - Detects firmware version
 * - Registers parameters with ROS
 * - Identifies applicable parameters
 * - Tracks behavior changes across firmware versions
 *
 * The tests use MockUbxTransceiver to simulate device communication without
 * requiring actual USB hardware, making the tests more reliable and faster.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <fstream>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/ubx_transceiver.hpp"
#include "ublox_dgnss_node/ubx/mock_ubx_transceiver.hpp"
#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_handler.hpp"
#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_parameter.hpp"
#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_parameter_loader.hpp"
#include "ublox_dgnss_node/ubx/mon/ubx_mon_ver.hpp"

using testing::Return;
using testing::_;

namespace ubx
{

// Implement the MockUbxTransceiver constructor since it's declared in the header but not defined
MockUbxTransceiver::MockUbxTransceiver()
{
  // Initialize with default values
  last_sent_message_ = nullptr;
}

namespace cfg
{
namespace test
{

/**
 * @brief Test fixture for UbxCfgHandler tests
 */
class UbxCfgHandlerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a temp directory for test files
    temp_dir_ = std::filesystem::temp_directory_path() / "ubx_cfg_test";
    std::cout << "Creating test directory at: " << temp_dir_.string() << std::endl;
    std::filesystem::create_directories(temp_dir_);
    
    // Path to a test TOML file we'll create
    test_toml_path_ = temp_dir_ / "test_parameters.toml";
    std::cout << "Test TOML file path: " << test_toml_path_.string() << std::endl;
    
    // Create a simple test TOML file with parameters
    std::string toml_content = createTestParameterToml();
    
    std::ofstream file(test_toml_path_);
    file << toml_content;
    file.close();
    
    // Verify file was written correctly
    std::ifstream check_file(test_toml_path_);
    std::string file_contents;
    if (check_file.is_open()) {
      std::stringstream buffer;
      buffer << check_file.rdbuf();
      file_contents = buffer.str();
      std::cout << "Successfully wrote test file with size: " << file_contents.size() << " bytes" << std::endl;
      check_file.close();
    } else {
      std::cout << "ERROR: Could not verify file contents - file not found or couldn't be opened" << std::endl;
    }
    
    // Initialize rclcpp if not already initialized
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    
    // Create a ROS node for testing
    node_ = std::make_shared<rclcpp::Node>("test_node");
    
    // Create a mock UBX transceiver
    mock_transceiver_ = std::make_shared<MockUbxTransceiver>();
    
    // Set up the mock transceiver behavior for initialization
    setupMockTransceiverForInit();
    
    // Create the UBX CFG handler with the mock transceiver
    handler_ = std::make_unique<UbxCfgHandler>(
      node_.get(),
      mock_transceiver_,
      "ZED-F9P",
      test_toml_path_.string());
  }

  void TearDown() override
  {
    // Clean up the temp directory and files
    std::filesystem::remove_all(temp_dir_);
    
    // Clean up the handler and node
    handler_.reset();
    node_.reset();
    
    // Shutdown rclcpp if we initialized it
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  /**
   * @brief Create test parameter TOML content
   * @return TOML content as string
   */
  std::string createTestParameterToml()
  {
    // Create a simple but valid TOML file for testing
    return R"(# UBX-CFG Test Parameters File
# TOML version for testing the UbxCfgHandler

version = "1.0.0"
device_types = ["ZED-F9P", "ZED-F9R"]

# Firmware version information
[firmware_versions]

# ZED-F9P firmware versions
[[firmware_versions.ZED-F9P]]
version = "HPG 1.00"
description = "Initial release"
release_date = "2021-01-01"

[[firmware_versions.ZED-F9P]]
version = "HPG 1.10"
description = "Feature update"
release_date = "2021-06-01"

[[firmware_versions.ZED-F9P]]
version = "HPG 1.13"
description = "Maintenance update"
release_date = "2021-09-01"

# ZED-F9R firmware versions
[[firmware_versions.ZED-F9R]]
version = "HPG 1.00"
description = "Initial release"
release_date = "2021-01-01"

[[firmware_versions.ZED-F9R]]
version = "HPG 1.10"
description = "Feature update"
release_date = "2021-06-01"

# Parameters configuration
[[parameters]]
name = "CFG_UART1_BAUDRATE"
key_id = "0x40520001"
type = "U4"
scale = 1.0
unit = "NA"
applicable_devices = ["ZED-F9P", "ZED-F9R"]
description = "Standard UART 1 Baudrate"
group = "UART1"
default_value = "0x00040800"

[parameters.firmware_support.ZED-F9P]
since = "HPG 1.00"

[[parameters.firmware_support.ZED-F9P.behavior_changes]]
version = "HPG 1.10"
description = "Support for higher baudrates"

[parameters.firmware_support.ZED-F9R]
since = "HPG 1.00"

[[parameters]]
name = "CFG_MSGOUT_UBX_NAV_PVT_UART1"
key_id = "0x20910006"
type = "U1"
scale = 1.0
unit = "NA"
applicable_devices = ["ZED-F9P", "ZED-F9R"]
description = "Output rate of the UBX-NAV-PVT message on port UART1"
group = "MSGOUT"
default_value = "0x01"

[parameters.firmware_support.ZED-F9P]
since = "HPG 1.00"

[parameters.firmware_support.ZED-F9R]
since = "HPG 1.00"

[[parameters]]
name = "CFG_MSGOUT_UBX_NAV_PVT_USB"
key_id = "0x20910007"
type = "U1"
scale = 1.0
unit = "NA"
applicable_devices = ["ZED-F9P"]
description = "Output rate of the UBX-NAV-PVT message on port USB"
group = "MSGOUT"
default_value = "0x01"

[parameters.firmware_support.ZED-F9P]
since = "HPG 1.00"

[[parameters]]
name = "CFG_NAVSPG_DYNMODEL"
key_id = "0x20110021"
type = "E1"
scale = 1.0
unit = "NA"
applicable_devices = ["ZED-F9P", "ZED-F9R"]
description = "Dynamic platform model"
group = "NAVSPG"
default_value = "0x00"

[parameters.possible_values]
DYN_MODEL_PORT = "0x00"
DYN_MODEL_STATIONARY = "0x02"
DYN_MODEL_PEDESTRIAN = "0x03"
DYN_MODEL_AUTOMOTIVE = "0x04"
DYN_MODEL_SEA = "0x05"
DYN_MODEL_AIRBORNE_1G = "0x06"
DYN_MODEL_AIRBORNE_2G = "0x07"
DYN_MODEL_AIRBORNE_4G = "0x08"
DYN_MODEL_WRIST = "0x09"

[parameters.firmware_support.ZED-F9P]
since = "HPG 1.00"

[parameters.firmware_support.ZED-F9R]
since = "HPG 1.00"
)";
  }

  /**
   * @brief Set up the mock transceiver for initialization
   * 
   * This sets up the mock transceiver to respond to the version query
   * and open/close methods during UbxCfgHandler initialization.
   */
  void setupMockTransceiverForInit()
  {
    // Set up mock for is_open, open, and close
    EXPECT_CALL(*mock_transceiver_, is_open())
      .WillRepeatedly(Return(true));
    EXPECT_CALL(*mock_transceiver_, open())
      .WillRepeatedly(Return(true));
    EXPECT_CALL(*mock_transceiver_, close())
      .Times(testing::AnyNumber());
      
    // Set up mock for version query response using UBX-MON-VER message
    EXPECT_CALL(*mock_transceiver_, write(testing::Property(&std::shared_ptr<const Frame>::get,
      testing::AllOf(
        testing::Field(&Frame::msg_class, ubx::UBX_MON),
        testing::Field(&Frame::msg_id, ubx::UBX_MON_VER)
      ))))
    .WillRepeatedly(Return(ubx::WriteResult{ubx::AckNack::ACK}));
      
    // Mock response for read operations, especially for UBX-MON-VER
    ON_CALL(*mock_transceiver_, read(testing::_, testing::_))
    .WillByDefault([this](std::shared_ptr<ubx::Frame> & out_frame, int /* timeout_ms */) {
        // Create an appropriate response for UBX-MON-VER
        frame = std::make_shared<Frame>();
        frame->msg_class = ubx::UBX_MON;
        frame->msg_id = ubx::UBX_MON_VER;
        
        // Create a proper UBX frame with header, payload, and footer
        // Start with the sync chars
        frame->buf.clear();
        frame->buf.push_back(0xB5);  // SYNC_CHAR_1
        frame->buf.push_back(0x62);  // SYNC_CHAR_2
        
        // Add class and ID
        frame->buf.push_back(ubx::UBX_MON);  // class
        frame->buf.push_back(ubx::UBX_MON_VER);  // id
        
        // Prepare the payload data
        std::vector<uint8_t> payload;
        
        // Software version (30 bytes, null-terminated)
        std::string sw_version = "RE 1.00";
        payload.insert(payload.end(), sw_version.begin(), sw_version.end());
        payload.resize(payload.size() + (30 - sw_version.size()), 0);  // Pad to 30 bytes with nulls
        
        // Hardware version (10 bytes, null-terminated)
        std::string hw_version = "00080000";
        payload.insert(payload.end(), hw_version.begin(), hw_version.end());
        payload.resize(payload.size() + (10 - hw_version.size()), 0);  // Pad to 10 bytes with nulls
        
        // Extension strings (each 30 bytes, null-terminated)
        std::string ext1 = "FWVER=RE 1.00";
        payload.insert(payload.end(), ext1.begin(), ext1.end());
        payload.resize(payload.size() + (30 - ext1.size()), 0);  // Pad to 30 bytes with nulls
        
        std::string ext2 = "PROTVER=27.30";
        payload.insert(payload.end(), ext2.begin(), ext2.end());
        payload.resize(payload.size() + (30 - ext2.size()), 0);  // Pad to 30 bytes with nulls
        
        // Add payload length (2 bytes, little-endian)
        uint16_t payload_length = payload.size();
        frame->buf.push_back(payload_length & 0xFF);  // Low byte first (little-endian)
        frame->buf.push_back((payload_length >> 8) & 0xFF);  // High byte
        
        // Add payload
        frame->buf.insert(frame->buf.end(), payload.begin(), payload.end());
        
        // Calculate and add checksum
        uint8_t ck_a = 0, ck_b = 0;
        for (size_t i = 2; i < frame->buf.size(); i++) {
            ck_a = (ck_a + frame->buf[i]) & 0xFF;
            ck_b = (ck_b + ck_a) & 0xFF;
        }
        frame->buf.push_back(ck_a);
        frame->buf.push_back(ck_b);
        
        // Set up the frame properties
        frame->payload = frame->buf.data() + 6;  // Payload starts after header (sync, class, id, len)
        frame->length = payload_length;
        
        // Copy our frame to the output frame
        out_frame = frame;
        
        return ubx::ReadResult{ubx::ReadStatus::SUCCESS};
      });
  }

  // Test directory and files
  std::filesystem::path temp_dir_;
  std::filesystem::path test_toml_path_;   ///< Path to test parameter file
  // ROS node
  std::shared_ptr<rclcpp::Node> node_;
  // Mock transceiver
  std::shared_ptr<MockUbxTransceiver> mock_transceiver_;
  // UBX CFG handler
  std::unique_ptr<UbxCfgHandler> handler_;
  // Frame for storing responses in mocks
  std::shared_ptr<Frame> frame;
};

/**
 * @brief Test initialization of UbxCfgHandler
 * 
 * Verifies that the UbxCfgHandler initializes correctly, including
 * firmware version detection and parameter registration.
 */
TEST_F(UbxCfgHandlerTest, Initialize)
{
  std::cout << "\n============== UbxCfgHandler Initialize Test ==============" << std::endl;
  
  // Step 1: Initialize the handler
  std::cout << "Initializing handler..." << std::endl;
  bool init_result = handler_->initialize();
  ASSERT_TRUE(init_result) << "Handler initialization failed";
  std::cout << "Handler initialized successfully" << std::endl;
  
  // Step 2: Check device type
  std::string device_type = handler_->get_device_type();
  std::cout << "Device type: " << device_type << std::endl;
  ASSERT_EQ(device_type, "ZED-F9P") << "Device type mismatch";
  
  // Step 3: Check firmware version
  std::string firmware_version = handler_->get_firmware_version();
  std::cout << "Firmware version: " << firmware_version << std::endl;
  ASSERT_EQ(firmware_version, "RE 1.00") << "Firmware version mismatch";
  
  // Step 4: Check parameter applicability
  std::cout << "Checking parameter applicability..." << std::endl;
  
  // Test DYNMODEL parameter
  bool dynmodel_applicable = handler_->is_parameter_applicable("CFG-NAVSPG-DYNMODEL");
  std::cout << "CFG-NAVSPG-DYNMODEL applicable: " << (dynmodel_applicable ? "true" : "false") << std::endl;
  ASSERT_TRUE(dynmodel_applicable) << "DYNMODEL parameter should be applicable";
  
  // Test UTCSTD parameter
  bool utcstd_applicable = handler_->is_parameter_applicable("CFG-NAVSPG-UTCSTD");
  std::cout << "CFG-NAVSPG-UTCSTD applicable: " << (utcstd_applicable ? "true" : "false") << std::endl;
  ASSERT_TRUE(utcstd_applicable) << "UTCSTD parameter should be applicable";
  
  // Step 5: Check behavior change detection
  std::cout << "Checking behavior change detection..." << std::endl;
  std::string behavior_change = handler_->get_parameter_behavior_change("CFG-NAVSPG-DYNMODEL");
  std::cout << "Behavior change for CFG-NAVSPG-DYNMODEL: " << behavior_change << std::endl;
  ASSERT_FALSE(behavior_change.empty()) << "Expected behavior change but none detected";
  ASSERT_EQ(behavior_change, "Default value changed from 3 to 2") << "Behavior change mismatch";
  
  // Step 6: Check parameters collection
  std::cout << "Checking parameters collection..." << std::endl;
  auto parameters = handler_->get_applicable_parameters();
  std::cout << "Number of applicable parameters: " << parameters.size() << std::endl;
  
  // Print all parameters for debugging
  std::cout << "Available parameters:" << std::endl;
  for (const auto& param : parameters) {
    std::cout << "  - " << param.get_name() << std::endl;
  }
  
  // Only verify that we have parameters, not the exact count
  ASSERT_GT(parameters.size(), 0u) << "No parameters were registered";
  
  std::cout << "============== Test Completed ==============" << std::endl;
}

/**
 * @brief Test parameter access validation
 * 
 * Verifies that UbxCfgHandler correctly validates parameter access.
 */
TEST_F(UbxCfgHandlerTest, ParameterAccessValidation)
{
  std::cout << "\n============== UbxCfgHandler Parameter Access Validation Test ==============" << std::endl;
  
  // Initialize the handler
  ASSERT_TRUE(handler_->initialize());
  
  // Test that the parameters are registered and applicable
  bool dynmodel_applicable = handler_->is_parameter_applicable("CFG-NAVSPG-DYNMODEL");
  EXPECT_TRUE(dynmodel_applicable) << "CFG-NAVSPG-DYNMODEL should be applicable";
  
  bool utcstd_applicable = handler_->is_parameter_applicable("CFG-NAVSPG-UTCSTD");
  EXPECT_TRUE(utcstd_applicable) << "CFG-NAVSPG-UTCSTD should be applicable";
  
  // Test nonexistent parameter
  bool nonexistent_param = handler_->is_parameter_applicable("NONEXISTENT-PARAMETER");
  EXPECT_FALSE(nonexistent_param) << "Nonexistent parameter should not be applicable";
  
  // Get parameter value should fail for nonexistent parameter
  bool get_result = handler_->get_parameter_value("NONEXISTENT-PARAMETER");
  EXPECT_FALSE(get_result) << "Getting nonexistent parameter should fail";
  
  std::cout << "Parameter access validation successful" << std::endl;
  std::cout << "============== Test Completed ==============" << std::endl;
}

/**
 * @brief Test parameter metadata verification
 * 
 * Verifies that UbxCfgHandler correctly handles parameter metadata.
 */
TEST_F(UbxCfgHandlerTest, ParameterMetadataVerification)
{
  std::cout << "\n============== UbxCfgHandler Parameter Metadata Test ==============" << std::endl;
  
  // Initialize the handler
  ASSERT_TRUE(handler_->initialize());
  
  // Test parameter applicability
  bool dynmodel_applicable = handler_->is_parameter_applicable("CFG-NAVSPG-DYNMODEL");
  EXPECT_TRUE(dynmodel_applicable) << "CFG-NAVSPG-DYNMODEL should be applicable";
  
  bool utcstd_applicable = handler_->is_parameter_applicable("CFG-NAVSPG-UTCSTD");
  EXPECT_TRUE(utcstd_applicable) << "CFG-NAVSPG-UTCSTD should be applicable";
  
  // Test behavior changes detection
  std::string behavior_change = handler_->get_parameter_behavior_change("CFG-NAVSPG-DYNMODEL");
  EXPECT_EQ(behavior_change, "Default value changed from 3 to 2") 
    << "Behavior change for CFG-NAVSPG-DYNMODEL should mention default value change";
  
  // Test empty behavior change for parameter without changes
  std::string no_change = handler_->get_parameter_behavior_change("CFG-NAVSPG-UTCSTD");
  EXPECT_TRUE(no_change.empty()) << "No behavior changes expected for CFG-NAVSPG-UTCSTD";
  
  std::cout << "Parameter metadata verification successful" << std::endl;
  std::cout << "============== Test Completed ==============" << std::endl;
}

/**
 * @brief Test parameter device applicability
 * 
 * Verifies that UbxCfgHandler correctly identifies parameters applicable to different devices.
 */
TEST_F(UbxCfgHandlerTest, ParameterDeviceApplicability)
{
  std::cout << "\n============== UbxCfgHandler Device Applicability Test ==============" << std::endl;
  
  // Initialize the handler for ZED-F9P
  ASSERT_TRUE(handler_->initialize());
  
  // Verify that both test parameters are applicable to ZED-F9P
  bool dynmodel_applicable = handler_->is_parameter_applicable("CFG-NAVSPG-DYNMODEL");
  EXPECT_TRUE(dynmodel_applicable) << "CFG-NAVSPG-DYNMODEL should be applicable to ZED-F9P";
  
  bool utcstd_applicable = handler_->is_parameter_applicable("CFG-NAVSPG-UTCSTD");
  EXPECT_TRUE(utcstd_applicable) << "CFG-NAVSPG-UTCSTD should be applicable to ZED-F9P";
  
  // Create a separate node for the F9R handler to avoid parameter registration conflicts
  auto f9r_node = std::make_shared<rclcpp::Node>("test_node_f9r");
  
  // Create a new handler for ZED-F9R using the separate node
  auto f9r_mock_transceiver = std::make_shared<MockUbxTransceiver>();
  
  // Set up the mock transceiver expectations for the F9R handler
  EXPECT_CALL(*f9r_mock_transceiver, is_open())
    .WillRepeatedly(Return(true));
  EXPECT_CALL(*f9r_mock_transceiver, open())
    .WillRepeatedly(Return(true));
  EXPECT_CALL(*f9r_mock_transceiver, close())
    .Times(testing::AnyNumber());
  EXPECT_CALL(*f9r_mock_transceiver, write(testing::_))
    .WillRepeatedly(Return(ubx::WriteResult{ubx::AckNack::ACK}));
  EXPECT_CALL(*f9r_mock_transceiver, read(testing::_, testing::_))
    .WillRepeatedly([](std::shared_ptr<ubx::Frame> & out_frame, int /* timeout_ms */) {
      // Create a UBX-MON-VER response with firmware version RE 1.00
      auto frame = std::make_shared<Frame>();
      frame->msg_class = ubx::UBX_MON;
      frame->msg_id = ubx::UBX_MON_VER;
      
      // Set up the payload for UBX-MON-VER
      std::string sw_version = "EXT CORE 1.00 (12ab345)";
      std::string hw_version = "00080000";
      std::string ext1 = "FWVER=RE 1.00";
      std::string ext2 = "PROTVER=27.30";
      
      // Prepare payload
      frame->buf.clear();
      frame->buf.insert(frame->buf.end(), sw_version.begin(), sw_version.end());
      frame->buf.resize(frame->buf.size() + (30 - sw_version.size()), 0);
      frame->buf.insert(frame->buf.end(), hw_version.begin(), hw_version.end());
      frame->buf.resize(frame->buf.size() + (10 - hw_version.size()), 0);
      frame->buf.insert(frame->buf.end(), ext1.begin(), ext1.end());
      frame->buf.resize(frame->buf.size() + (30 - ext1.size()), 0);
      frame->buf.insert(frame->buf.end(), ext2.begin(), ext2.end());
      frame->buf.resize(frame->buf.size() + (30 - ext2.size()), 0);
      
      // Set the payload pointer
      frame->payload = frame->buf.data();
      frame->length = frame->buf.size();
      
      // Copy our frame to the output frame
      out_frame = frame;
      
      return ubx::ReadResult{ubx::ReadStatus::SUCCESS};
    });
  
  UbxCfgHandler f9r_handler(f9r_node.get(), f9r_mock_transceiver, "ZED-F9R", test_toml_path_.string());
  ASSERT_TRUE(f9r_handler.initialize());
  
  // DYNMODEL should be applicable to both device types
  bool dynmodel_f9r = f9r_handler.is_parameter_applicable("CFG-NAVSPG-DYNMODEL");
  EXPECT_TRUE(dynmodel_f9r) << "CFG-NAVSPG-DYNMODEL should be applicable to ZED-F9R";
  
  // UTCSTD should only be applicable to ZED-F9P, not ZED-F9R
  bool utcstd_f9r = f9r_handler.is_parameter_applicable("CFG-NAVSPG-UTCSTD");
  EXPECT_FALSE(utcstd_f9r) << "CFG-NAVSPG-UTCSTD should not be applicable to ZED-F9R";
  
  std::cout << "Parameter device applicability verification successful" << std::endl;
  std::cout << "============== Test Completed ==============" << std::endl;
}

/**
 * @brief Test handling of nonexistent parameter
 * 
 * Verifies that UbxCfgHandler correctly handles requests for
 * parameters that don't exist.
 */
TEST_F(UbxCfgHandlerTest, NonexistentParameter)
{
  std::cout << "\n============== UbxCfgHandler NonexistentParameter Test ==============" << std::endl;
  
  // Initialize the handler
  ASSERT_TRUE(handler_->initialize());
  
  // Try to get a nonexistent parameter
  bool get_result = handler_->get_parameter_value("NONEXISTENT-PARAMETER");
  EXPECT_FALSE(get_result) << "Getting nonexistent parameter should fail";
  
  // Try setting a nonexistent parameter
  rclcpp::ParameterValue param_value(std::string("0x2A")); // 0x2A = 42 in hex
  bool set_result = handler_->set_parameter_value("NONEXISTENT-PARAMETER", param_value);
  EXPECT_FALSE(set_result) << "Setting nonexistent parameter should fail";
  
  // Check if the parameter is applicable
  bool is_applicable = handler_->is_parameter_applicable("NONEXISTENT-PARAMETER");
  EXPECT_FALSE(is_applicable) << "Nonexistent parameter should not be applicable";
  
  std::cout << "Nonexistent parameter handling working as expected" << std::endl;
  std::cout << "============== Test Completed ==============" << std::endl;
}

}  // namespace test
}  // namespace cfg
}  // namespace ubx

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
