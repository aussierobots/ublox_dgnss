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
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_parameter.hpp"
#include "ublox_dgnss_node/ubx/ubx_types.hpp"

namespace
{

class UbxCfgParameterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a test parameter
    std::vector<std::string> applicable_devices = {"ZED-F9P", "ZED-F9R"};

    std::map<std::string, ubx::cfg::FirmwareSupport> firmware_support;

    // Add firmware support for ZED-F9P
    ubx::cfg::FirmwareSupport f9p_support;
    f9p_support.since = "HPG 1.13";

    // Add behavior change
    ubx::cfg::BehaviorChange change;
    change.version = "HPG 1.32";
    change.description = "Improved handling and performance";
    f9p_support.behavior_changes.push_back(change);

    firmware_support["ZED-F9P"] = f9p_support;

    // Add firmware support for ZED-F9R
    ubx::cfg::FirmwareSupport f9r_support;
    f9r_support.since = "HPS 1.13";
    f9r_support.until = "HPS 1.30";  // Deprecated in 1.30

    firmware_support["ZED-F9R"] = f9r_support;

    // Create possible values for enum type
    std::map<std::string, std::string> possible_values;
    possible_values["OPTION_1"] = "0x01";
    possible_values["OPTION_2"] = "0x02";

    // Create the parameter
    ubx::cfg::ubx_key_id_t key_id;
    key_id.all = 0x20920001;

    parameter_ = std::make_unique<ubx::cfg::UbxCfgParameter>(
      "CFG_TEST_PARAM",
      key_id,
      ubx::ubx_type_t::E1,
      1.0,
      ubx::cfg::ubx_unit_t::NA,
      applicable_devices,
      "Test parameter",
      "TEST",
      firmware_support,
      possible_values,
      "0x01",
      "0x00",
      "0x03"
    );
  }

  std::unique_ptr<ubx::cfg::UbxCfgParameter> parameter_;
};

// Test basic parameter properties
TEST_F(UbxCfgParameterTest, BasicProperties)
{
  EXPECT_EQ(parameter_->get_name(), "CFG_TEST_PARAM");
  EXPECT_EQ(parameter_->get_key_id().all, 0x20920001);
  EXPECT_EQ(parameter_->get_type(), ubx::ubx_type_t::E1);
  EXPECT_DOUBLE_EQ(parameter_->get_scale(), 1.0);
  EXPECT_EQ(parameter_->get_unit(), ubx::cfg::ubx_unit_t::NA);
  EXPECT_EQ(parameter_->get_description(), "Test parameter");
  EXPECT_EQ(parameter_->get_group(), "TEST");
  EXPECT_EQ(parameter_->get_default_value(), "0x01");

  // Check applicable devices
  auto devices = parameter_->get_applicable_devices();
  ASSERT_EQ(devices.size(), 2u);
  EXPECT_EQ(devices[0], "ZED-F9P");
  EXPECT_EQ(devices[1], "ZED-F9R");

  // Check possible values
  auto values = parameter_->get_possible_values();
  ASSERT_EQ(values.size(), 2u);
  EXPECT_EQ(values["OPTION_1"], "0x01");
  EXPECT_EQ(values["OPTION_2"], "0x02");
}

// Test firmware support
TEST_F(UbxCfgParameterTest, FirmwareSupport)
{
  // Check firmware support
  auto firmware_support = parameter_->get_firmware_support();
  ASSERT_EQ(firmware_support.size(), 2u);

  // Check ZED-F9P support
  ASSERT_TRUE(firmware_support.find("ZED-F9P") != firmware_support.end());
  auto f9p_support = firmware_support["ZED-F9P"];
  EXPECT_EQ(f9p_support.since, "HPG 1.13");
  EXPECT_FALSE(f9p_support.until.has_value());
  ASSERT_EQ(f9p_support.behavior_changes.size(), 1u);
  EXPECT_EQ(f9p_support.behavior_changes[0].version, "HPG 1.32");
  EXPECT_EQ(f9p_support.behavior_changes[0].description, "Improved handling and performance");

  // Check ZED-F9R support
  ASSERT_TRUE(firmware_support.find("ZED-F9R") != firmware_support.end());
  auto f9r_support = firmware_support["ZED-F9R"];
  EXPECT_EQ(f9r_support.since, "HPS 1.13");
  ASSERT_TRUE(f9r_support.until.has_value());
  EXPECT_EQ(f9r_support.until.value(), "HPS 1.30");
}

// Test device applicability
TEST_F(UbxCfgParameterTest, DeviceApplicability)
{
  EXPECT_TRUE(parameter_->is_applicable_to_device("ZED-F9P"));
  EXPECT_TRUE(parameter_->is_applicable_to_device("ZED-F9R"));
  EXPECT_FALSE(parameter_->is_applicable_to_device("ZED-F9T"));  // Non-existent device
}

// Test firmware version support
TEST_F(UbxCfgParameterTest, FirmwareVersionSupport)
{
  // ZED-F9P firmware support
  EXPECT_TRUE(parameter_->is_supported_in_firmware("ZED-F9P", "HPG 1.13"));  // Since version
  EXPECT_TRUE(parameter_->is_supported_in_firmware("ZED-F9P", "HPG 1.20"));  // Middle version
  EXPECT_TRUE(parameter_->is_supported_in_firmware("ZED-F9P", "HPG 1.32"));  // Latest version
  EXPECT_FALSE(parameter_->is_supported_in_firmware("ZED-F9P", "HPG 1.00"));  // Before since version

  // ZED-F9R firmware support
  EXPECT_TRUE(parameter_->is_supported_in_firmware("ZED-F9R", "HPS 1.13"));  // Since version
  EXPECT_TRUE(parameter_->is_supported_in_firmware("ZED-F9R", "HPS 1.20"));  // Middle version
  EXPECT_FALSE(parameter_->is_supported_in_firmware("ZED-F9R", "HPS 1.30"));  // Until version (deprecated)
  EXPECT_FALSE(parameter_->is_supported_in_firmware("ZED-F9R", "HPS 1.00"));  // Before since version

  // Non-existent device
  EXPECT_FALSE(parameter_->is_supported_in_firmware("ZED-F9T", "HPG 1.20"));
}

// Test behavior changes
TEST_F(UbxCfgParameterTest, BehaviorChanges)
{
  // ZED-F9P behavior changes
  EXPECT_EQ(parameter_->get_behavior_change("ZED-F9P", "HPG 1.13"), "");  // No change in this version
  EXPECT_EQ(parameter_->get_behavior_change("ZED-F9P", "HPG 1.31"), "");  // No change in this version
  EXPECT_EQ(parameter_->get_behavior_change("ZED-F9P", "HPG 1.32"),
      "Improved handling and performance");                                                                // Change in this version
  EXPECT_EQ(parameter_->get_behavior_change("ZED-F9P", "HPG 1.33"),
      "Improved handling and performance");                                                                // Change applies to later versions too

  // ZED-F9R behavior changes (none defined)
  EXPECT_EQ(parameter_->get_behavior_change("ZED-F9R", "HPS 1.20"), "");

  // Non-existent device
  EXPECT_EQ(parameter_->get_behavior_change("ZED-F9T", "HPG 1.20"), "");
}

// Test firmware version comparison
TEST_F(UbxCfgParameterTest, FirmwareVersionComparison)
{
  // Same firmware type, different versions
  EXPECT_LT(ubx::cfg::UbxCfgParameter::compare_firmware_versions("HPG 1.13", "HPG 1.20"), 0);
  EXPECT_GT(ubx::cfg::UbxCfgParameter::compare_firmware_versions("HPG 1.20", "HPG 1.13"), 0);
  EXPECT_EQ(ubx::cfg::UbxCfgParameter::compare_firmware_versions("HPG 1.20", "HPG 1.20"), 0);

  // Different firmware types
  EXPECT_NE(ubx::cfg::UbxCfgParameter::compare_firmware_versions("HPG 1.20", "HPS 1.20"), 0);

  // Major version differences
  EXPECT_LT(ubx::cfg::UbxCfgParameter::compare_firmware_versions("HPG 1.20", "HPG 2.00"), 0);
  EXPECT_GT(ubx::cfg::UbxCfgParameter::compare_firmware_versions("HPG 2.00", "HPG 1.20"), 0);
}

// Test value conversion
TEST_F(UbxCfgParameterTest, ValueConversion)
{
  // Test enum value conversion
  ubx::value_t value = parameter_->string_to_ubx_value("OPTION_1");
  EXPECT_EQ(value.u1, 0x01);

  value = parameter_->string_to_ubx_value("OPTION_2");
  EXPECT_EQ(value.u1, 0x02);

  value = parameter_->string_to_ubx_value("0x03");
  EXPECT_EQ(value.u1, 0x03);

  // Test invalid enum value
  EXPECT_THROW(parameter_->string_to_ubx_value("INVALID_OPTION"), std::invalid_argument);

  // Test string conversion back
  value.u1 = 0x01;
  EXPECT_EQ(parameter_->ubx_value_to_string(value), "OPTION_1");

  value.u1 = 0x02;
  EXPECT_EQ(parameter_->ubx_value_to_string(value), "OPTION_2");

  value.u1 = 0x03;
  EXPECT_EQ(parameter_->ubx_value_to_string(value), "0x03");
}

// Test value validation
TEST_F(UbxCfgParameterTest, ValueValidation)
{
  // Valid values
  EXPECT_TRUE(parameter_->validate_string_value("OPTION_1"));
  EXPECT_TRUE(parameter_->validate_string_value("OPTION_2"));
  EXPECT_TRUE(parameter_->validate_string_value("0x01"));
  EXPECT_TRUE(parameter_->validate_string_value("0x02"));
  EXPECT_TRUE(parameter_->validate_string_value("0x03"));  // Max value

  // Invalid values
  EXPECT_FALSE(parameter_->validate_string_value("INVALID_OPTION"));
  EXPECT_FALSE(parameter_->validate_string_value("0x04"));  // Above max
  EXPECT_FALSE(parameter_->validate_string_value("-1"));  // Below min
}

}  // namespace

// Run all tests
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
