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
 * @file test_ubx_cfg_parameter.cpp
 * @brief Unit tests for the UbxCfgParameter class
 */

#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <map>
#include <optional>

// Only include the parameter header, which already includes the item header
#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_parameter.hpp"
#include "ublox_dgnss_node/ubx/ubx_types.hpp"

namespace ubx {
namespace cfg {
namespace test {

/**
 * @brief Test fixture for UbxCfgParameter
 */
class UbxCfgParameterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create dynamic platform model parameter (CFG-NAVSPG-DYNMODEL)
    // This parameter uses enum values that must be handled in a consistent format
    std::vector<std::string> applicable_devices = {"ZED-F9P", "ZED-F9R"};
    std::map<std::string, FirmwareSupport> firmware_support;
    
    FirmwareSupport f9p_support;
    f9p_support.since = "HPG 1.00";
    
    BehaviorChange behavior_change;
    behavior_change.version = "HPG 1.13";
    behavior_change.description = "Default value changed from 3 to 2";
    f9p_support.behavior_changes.push_back(behavior_change);
    
    firmware_support["ZED-F9P"] = f9p_support;
    firmware_support["ZED-F9R"] = f9p_support;
    
    std::map<std::string, std::string> possible_values;
    // Use original enum identifiers from the UBX header file with hex values
    // This ensures consistency with how values are defined in the firmware
    possible_values["DYN_MODEL_PORT"] = "0x00";
    possible_values["DYN_MODEL_STAT"] = "0x02";
    possible_values["DYN_MODEL_PED"] = "0x03";
    possible_values["DYN_MODEL_AUTOMOT"] = "0x04";
    possible_values["DYN_MODEL_SEA"] = "0x05";
    possible_values["DYN_MODEL_AIR1"] = "0x06";
    possible_values["DYN_MODEL_AIR2"] = "0x07";
    possible_values["DYN_MODEL_AIR4"] = "0x08";
    
    ubx_key_id_t key_id;
    key_id.all = 0x20510010;
    
    parameter_ = std::make_unique<UbxCfgParameter>(
      "CFG-NAVSPG-DYNMODEL",
      key_id,
      ubx::E1,
      1.0,
      NA,
      applicable_devices,
      "Dynamic platform model",
      "NAVSPG",
      firmware_support,
      possible_values,
      "2",     // Default value
      "0",     // Min value
      "0x08"   // Max value must be in hex format to match the format in possible_values
    );
  }
  
  std::unique_ptr<UbxCfgParameter> parameter_;
};

/**
 * @brief Test basic properties of the parameter
 */
TEST_F(UbxCfgParameterTest, BasicProperties)
{
  EXPECT_EQ(parameter_->get_name(), "CFG-NAVSPG-DYNMODEL");
  
  ubx_key_id_t key_id = parameter_->get_key_id();
  EXPECT_EQ(key_id.all, 0x20510010);
  
  EXPECT_EQ(parameter_->get_type(), ubx::E1);
  EXPECT_DOUBLE_EQ(parameter_->get_scale(), 1.0);
  EXPECT_EQ(parameter_->get_unit(), NA);
  EXPECT_EQ(parameter_->get_description(), "Dynamic platform model");
  EXPECT_EQ(parameter_->get_group(), "NAVSPG");
  
  // Check default, min, max values
  EXPECT_EQ(parameter_->get_min_value(), "0");
  EXPECT_EQ(parameter_->get_max_value(), "0x08");
  EXPECT_EQ(parameter_->get_default_value(), "2");
  
  // Check applicable devices
  auto devices = parameter_->get_applicable_devices();
  ASSERT_EQ(devices.size(), 2u);
  EXPECT_EQ(devices[0], "ZED-F9P");
  EXPECT_EQ(devices[1], "ZED-F9R");
  
  // Check possible values
  auto values = parameter_->get_possible_values();
  ASSERT_EQ(values.size(), 8u);
  EXPECT_EQ(values.at("DYN_MODEL_PORT"), "0x00");
  EXPECT_EQ(values.at("DYN_MODEL_STAT"), "0x02");
  EXPECT_EQ(values.at("DYN_MODEL_PED"), "0x03");
}

/**
 * @brief Test device applicability checks
 */
TEST_F(UbxCfgParameterTest, DeviceApplicability)
{
  EXPECT_TRUE(parameter_->is_applicable_to_device("ZED-F9P"));
  EXPECT_TRUE(parameter_->is_applicable_to_device("ZED-F9R"));
  EXPECT_FALSE(parameter_->is_applicable_to_device("ZED-F9T"));  // Non-existent device
  EXPECT_FALSE(parameter_->is_applicable_to_device(""));  // Empty string
}

/**
 * @brief Test firmware version support
 */
TEST_F(UbxCfgParameterTest, FirmwareVersionSupport)
{
  // Test firmware version support for ZED-F9P
  EXPECT_TRUE(parameter_->is_supported_in_firmware("ZED-F9P", "HPG 1.13"));
  EXPECT_TRUE(parameter_->is_supported_in_firmware("ZED-F9P", "HPG 1.32"));
  EXPECT_TRUE(parameter_->is_supported_in_firmware("ZED-F9P", "HPG 1.10"));  // Support starts at HPG 1.00
  
  // Test firmware version support for ZED-F9R (no deprecation in our test setup)
  EXPECT_TRUE(parameter_->is_supported_in_firmware("ZED-F9R", "HPG 1.20"));
  EXPECT_TRUE(parameter_->is_supported_in_firmware("ZED-F9R", "HPG 1.30"));
  EXPECT_TRUE(parameter_->is_supported_in_firmware("ZED-F9R", "HPG 1.40"));
  EXPECT_TRUE(parameter_->is_supported_in_firmware("ZED-F9R", "HPG 1.50"));
  EXPECT_TRUE(parameter_->is_supported_in_firmware("ZED-F9R", "HPG 1.10"));
  
  // Test invalid device
  EXPECT_FALSE(parameter_->is_supported_in_firmware("ZED-F10P", "HPG 1.32"));
}

/**
 * @brief Test behavior change detection
 */
TEST_F(UbxCfgParameterTest, BehaviorChanges)
{
  // Test behavior changes for ZED-F9P
  EXPECT_EQ(parameter_->get_behavior_change("ZED-F9P", "HPG 1.12"), "");  // Before behavior change
  EXPECT_EQ(parameter_->get_behavior_change("ZED-F9P", "HPG 1.13"), "Default value changed from 3 to 2");  // At behavior change
  EXPECT_EQ(parameter_->get_behavior_change("ZED-F9P", "HPG 1.20"), "Default value changed from 3 to 2");  // After behavior change
  
  // Test behavior changes for ZED-F9R (same behavior changes in our setup)
  EXPECT_EQ(parameter_->get_behavior_change("ZED-F9R", "HPG 1.12"), "");  // Before behavior change
  EXPECT_EQ(parameter_->get_behavior_change("ZED-F9R", "HPG 1.13"), "Default value changed from 3 to 2");  // At behavior change
  
  // Test behavior changes for unsupported device
  EXPECT_EQ(parameter_->get_behavior_change("ZED-F9T", "HPG 1.32"), "");
}

/**
 * @brief Test firmware version comparison
 */
TEST_F(UbxCfgParameterTest, FirmwareVersionComparison)
{
  // Same firmware type, different versions
  EXPECT_LT(UbxCfgParameter::compare_firmware_versions("HPG 1.13", "HPG 1.20"), 0);
  EXPECT_GT(UbxCfgParameter::compare_firmware_versions("HPG 1.20", "HPG 1.13"), 0);
  EXPECT_EQ(UbxCfgParameter::compare_firmware_versions("HPG 1.20", "HPG 1.20"), 0);
  
  // Different major versions
  EXPECT_LT(UbxCfgParameter::compare_firmware_versions("HPG 1.20", "HPG 2.10"), 0);
  EXPECT_GT(UbxCfgParameter::compare_firmware_versions("HPG 2.10", "HPG 1.20"), 0);
  
  // Different firmware types
  EXPECT_NE(UbxCfgParameter::compare_firmware_versions("HPG 1.20", "HPS 1.20"), 0);
}

/**
 * @brief Test value conversion
 */
TEST_F(UbxCfgParameterTest, ValueConversion)
{
  // For an E1 type, ubx_value_to_string should return the enum name
  ubx::value_t value3;
  value3.u1 = 3;  // DYN_MODEL_PED value
  EXPECT_EQ(parameter_->ubx_value_to_string(value3), "DYN_MODEL_PED");
  
  // Test another value
  ubx::value_t value0;
  value0.u1 = 0;  // DYN_MODEL_PORT value
  EXPECT_EQ(parameter_->ubx_value_to_string(value0), "DYN_MODEL_PORT");
}

/**
 * @brief Test value validation for enum values
 * 
 * This test verifies that:
 * 1. All valid enum values (0, 2-8) are properly accepted
 * 2. Invalid values (9, 10, 255) are properly rejected
 * 
 * Note: All enum values are handled in hex format internally
 */
TEST_F(UbxCfgParameterTest, ValueValidation)
{
  // Create vectors of values to test for cleaner organization
  std::vector<uint8_t> expected_valid_values = {0, 2, 3, 4, 5, 6, 7, 8};
  std::vector<uint8_t> expected_invalid_values = {9, 10, 255};

  // Test valid values
  for (auto val : expected_valid_values) {
    ubx::value_t test_value;
    test_value.u1 = val;
    bool result = parameter_->validate_ubx_value(test_value);
    EXPECT_TRUE(result) << "Value " << static_cast<int>(val) << " should be valid";
  }

  // Test invalid values
  for (auto val : expected_invalid_values) {
    ubx::value_t test_value;
    test_value.u1 = val;
    bool result = parameter_->validate_ubx_value(test_value);
    EXPECT_FALSE(result) << "Value " << static_cast<int>(val) << " should be invalid";
  }
}

/**
 * @brief Test constructor with minimal parameters
 */
TEST_F(UbxCfgParameterTest, MinimalConstructor)
{
  // Create a parameter with minimal constructor parameters
  std::vector<std::string> devices = {"ZED-F9P"};
  std::map<std::string, FirmwareSupport> firmware_support;
  FirmwareSupport support;
  support.since = "HPG 1.00";
  firmware_support["ZED-F9P"] = support;
  
  ubx_key_id_t key_id;
  key_id.all = 0x20510011;
  
  // Create an empty map for possible values
  std::map<std::string, std::string> empty_possible_values;
  
  UbxCfgParameter minimal_param(
    "CFG-MINIMAL-TEST",
    key_id,
    ubx::U1,  // Use correct namespace
    1.0,
    NA,
    devices,
    "Minimal test parameter",
    "TEST",
    firmware_support,
    empty_possible_values  // Pass empty map explicitly
  );
  
  // Check that default values were used for optional parameters
  EXPECT_TRUE(minimal_param.get_possible_values().empty());
  EXPECT_EQ(minimal_param.get_default_value(), "");
  EXPECT_FALSE(minimal_param.get_min_value().has_value());
  EXPECT_FALSE(minimal_param.get_max_value().has_value());
}

}  // namespace test
}  // namespace cfg
}  // namespace ubx

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
