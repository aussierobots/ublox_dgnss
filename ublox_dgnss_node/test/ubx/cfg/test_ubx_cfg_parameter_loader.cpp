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
 * @file test_ubx_cfg_parameter_loader.cpp
 * @brief Unit tests for the UbxCfgParameterLoader class
 */

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <filesystem>

#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_parameter_loader.hpp"
#include "ublox_dgnss_node/ubx/ubx_types.hpp"

namespace
{

class UbxCfgParameterLoaderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a temporary test directory
    test_dir_ = std::filesystem::temp_directory_path() / "ubx_cfg_test";
    std::filesystem::create_directories(test_dir_);
    
    // Create a test parameter file
    test_file_ = test_dir_ / "test_parameters.json";
    createTestParameterFile(test_file_);
    
    // Create a parameter loader
    loader_ = std::make_unique<ubx::cfg::UbxCfgParameterLoader>(test_file_.string());
  }

  void TearDown() override
  {
    // Clean up test files
    std::filesystem::remove_all(test_dir_);
  }

  void createTestParameterFile(const std::filesystem::path & file_path)
  {
    // Create a test parameter file with firmware version support
    std::ofstream file(file_path);
    file << R"({
      "version": "1.0.0",
      "device_types": ["ZED-F9P", "ZED-F9R"],
      "firmware_versions": {
        "ZED-F9P": [
          {
            "version": "HPG 1.13",
            "description": "Early production release",
            "release_date": "2020-06-15"
          },
          {
            "version": "HPG 1.30",
            "description": "Production release with improved RTK performance",
            "release_date": "2022-01-15"
          },
          {
            "version": "HPG 1.32",
            "description": "Latest production release with enhanced RTK performance",
            "release_date": "2024-01-14"
          }
        ],
        "ZED-F9R": [
          {
            "version": "HPS 1.13",
            "description": "Early production release",
            "release_date": "2020-08-10"
          },
          {
            "version": "HPS 1.20",
            "description": "Production release with improved dead reckoning",
            "release_date": "2022-03-10"
          },
          {
            "version": "HPS 1.30",
            "description": "Latest production release with enhanced dead reckoning",
            "release_date": "2024-01-14"
          }
        ]
      },
      "parameters": [
        {
          "name": "CFG_UART1_BAUDRATE",
          "key_id": "0x40520001",
          "type": "U4",
          "scale": 1.0,
          "unit": "NA",
          "applicable_devices": ["ZED-F9P", "ZED-F9R"],
          "description": "UART1 baud rate",
          "group": "UART",
          "firmware_support": {
            "ZED-F9P": {
              "since": "HPG 1.13"
            },
            "ZED-F9R": {
              "since": "HPS 1.13"
            }
          },
          "min_value": "9600",
          "max_value": "921600",
          "default_value": "38400"
        },
        {
          "name": "CFG_RATE_MEAS",
          "key_id": "0x30210001",
          "type": "U2",
          "scale": 0.001,
          "unit": "S",
          "applicable_devices": ["ZED-F9P", "ZED-F9R"],
          "description": "Measurement rate",
          "group": "RATE",
          "firmware_support": {
            "ZED-F9P": {
              "since": "HPG 1.13",
              "behavior_changes": [
                {
                  "version": "HPG 1.32",
                  "description": "Improved handling and performance"
                }
              ]
            },
            "ZED-F9R": {
              "since": "HPS 1.13"
            }
          },
          "min_value": "50",
          "max_value": "10000",
          "default_value": "1000"
        },
        {
          "name": "CFG_NAVSPG_DYNMODEL",
          "key_id": "0x20110021",
          "type": "E1",
          "scale": 1.0,
          "unit": "NA",
          "applicable_devices": ["ZED-F9P", "ZED-F9R"],
          "description": "Dynamic platform model",
          "group": "NAVSPG",
          "firmware_support": {
            "ZED-F9P": {
              "since": "HPG 1.30"
            },
            "ZED-F9R": {
              "since": "HPS 1.20"
            }
          },
          "possible_values": {
            "DYN_MODEL_PORT": "0x00",
            "DYN_MODEL_STATIONARY": "0x02",
            "DYN_MODEL_PEDESTRIAN": "0x03",
            "DYN_MODEL_AUTOMOTIVE": "0x04",
            "DYN_MODEL_SEA": "0x05",
            "DYN_MODEL_AIRBORNE_1G": "0x06",
            "DYN_MODEL_AIRBORNE_2G": "0x07",
            "DYN_MODEL_AIRBORNE_4G": "0x08",
            "DYN_MODEL_WRIST": "0x09"
          },
          "default_value": "0x04"
        },
        {
          "name": "CFG_MSGOUT_RTCM_3X_TYPE1005_USB",
          "key_id": "0x209102bd",
          "type": "U1",
          "scale": 1.0,
          "unit": "NA",
          "applicable_devices": ["ZED-F9P", "ZED-F9R"],
          "description": "RTCM 3X Type 1005 message output rate on USB",
          "group": "MSGOUT",
          "firmware_support": {
            "ZED-F9P": {
              "since": "HPG 1.13",
              "until": "HPG 1.32"
            },
            "ZED-F9R": {
              "since": "HPS 1.13",
              "until": "HPS 1.30"
            }
          },
          "min_value": "0",
          "max_value": "255",
          "default_value": "0"
        }
      ]
    })";
    file.close();
  }

  std::filesystem::path test_dir_;
  std::filesystem::path test_file_;
  std::unique_ptr<ubx::cfg::UbxCfgParameterLoader> loader_;
};

// Test loading parameters
TEST_F(UbxCfgParameterLoaderTest, LoadParameters)
{
  // Load parameters
  EXPECT_TRUE(loader_->load());
  
  // Check the number of parameters
  auto parameters = loader_->get_all_parameters();
  EXPECT_EQ(parameters.size(), 4u);
}

// Test parameter lookup by name
TEST_F(UbxCfgParameterLoaderTest, ParameterLookupByName)
{
  // Load parameters
  EXPECT_TRUE(loader_->load());
  
  // Look up a parameter by name
  auto param = loader_->get_parameter_by_name("CFG_UART1_BAUDRATE");
  ASSERT_TRUE(param.has_value());
  EXPECT_EQ(param->get_name(), "CFG_UART1_BAUDRATE");
  EXPECT_EQ(param->get_key_id().all, 0x40520001);
  
  // Look up a non-existent parameter
  auto non_existent = loader_->get_parameter_by_name("NON_EXISTENT");
  EXPECT_FALSE(non_existent.has_value());
}

// Test parameter lookup by key ID
TEST_F(UbxCfgParameterLoaderTest, ParameterLookupByKeyId)
{
  // Load parameters
  EXPECT_TRUE(loader_->load());
  
  // Look up a parameter by key ID
  ubx::cfg::ubx_key_id_t key_id;
  key_id.all = 0x40520001;
  auto param = loader_->get_parameter_by_key_id(key_id);
  ASSERT_TRUE(param.has_value());
  EXPECT_EQ(param->get_name(), "CFG_UART1_BAUDRATE");
  EXPECT_EQ(param->get_key_id().all, 0x40520001);
  
  // Look up a non-existent parameter
  ubx::cfg::ubx_key_id_t non_existent_key;
  non_existent_key.all = 0x12345678;
  auto non_existent = loader_->get_parameter_by_key_id(non_existent_key);
  EXPECT_FALSE(non_existent.has_value());
}

// Test filtering parameters by device type
TEST_F(UbxCfgParameterLoaderTest, FilterByDeviceType)
{
  // Load parameters
  EXPECT_TRUE(loader_->load());
  
  // Get parameters for ZED-F9P
  auto f9p_params = loader_->get_parameters_for_device("ZED-F9P");
  EXPECT_EQ(f9p_params.size(), 4u);
  
  // Get parameters for ZED-F9R
  auto f9r_params = loader_->get_parameters_for_device("ZED-F9R");
  EXPECT_EQ(f9r_params.size(), 4u);
  
  // Get parameters for non-existent device
  auto non_existent = loader_->get_parameters_for_device("ZED-F9T");
  EXPECT_EQ(non_existent.size(), 0u);
}

// Test filtering parameters by device type and firmware version
TEST_F(UbxCfgParameterLoaderTest, FilterByDeviceAndFirmware)
{
  // Load parameters
  EXPECT_TRUE(loader_->load());
  
  // Get parameters for ZED-F9P with early firmware
  auto f9p_early = loader_->get_parameters_for_device_and_firmware("ZED-F9P", "HPG 1.13");
  EXPECT_EQ(f9p_early.size(), 3u);  // UART, RATE, and RTCM parameters
  
  // Get parameters for ZED-F9P with middle firmware
  auto f9p_middle = loader_->get_parameters_for_device_and_firmware("ZED-F9P", "HPG 1.30");
  EXPECT_EQ(f9p_middle.size(), 4u);  // UART, RATE, NAVSPG, and RTCM parameters
  
  // Get parameters for ZED-F9P with latest firmware
  auto f9p_latest = loader_->get_parameters_for_device_and_firmware("ZED-F9P", "HPG 1.32");
  EXPECT_EQ(f9p_latest.size(), 3u);  // UART, RATE, NAVSPG parameters (RTCM deprecated)
  
  // Get parameters for ZED-F9R with early firmware
  auto f9r_early = loader_->get_parameters_for_device_and_firmware("ZED-F9R", "HPS 1.13");
  EXPECT_EQ(f9r_early.size(), 3u);  // UART, RATE, and RTCM parameters
  
  // Get parameters for ZED-F9R with middle firmware
  auto f9r_middle = loader_->get_parameters_for_device_and_firmware("ZED-F9R", "HPS 1.20");
  EXPECT_EQ(f9r_middle.size(), 4u);  // UART, RATE, NAVSPG, and RTCM parameters
  
  // Get parameters for ZED-F9R with latest firmware
  auto f9r_latest = loader_->get_parameters_for_device_and_firmware("ZED-F9R", "HPS 1.30");
  EXPECT_EQ(f9r_latest.size(), 3u);  // UART, RATE, NAVSPG parameters (RTCM deprecated)
}

// Test getting available device types
TEST_F(UbxCfgParameterLoaderTest, AvailableDeviceTypes)
{
  // Load parameters
  EXPECT_TRUE(loader_->load());
  
  // Get available device types
  auto device_types = loader_->get_available_device_types();
  ASSERT_EQ(device_types.size(), 2u);
  EXPECT_EQ(device_types[0], "ZED-F9P");
  EXPECT_EQ(device_types[1], "ZED-F9R");
}

// Test getting available firmware versions
TEST_F(UbxCfgParameterLoaderTest, AvailableFirmwareVersions)
{
  // Load parameters
  EXPECT_TRUE(loader_->load());
  
  // Get available firmware versions for ZED-F9P
  auto f9p_versions = loader_->get_available_firmware_versions("ZED-F9P");
  ASSERT_EQ(f9p_versions.size(), 3u);
  EXPECT_EQ(f9p_versions[0], "HPG 1.13");
  EXPECT_EQ(f9p_versions[1], "HPG 1.30");
  EXPECT_EQ(f9p_versions[2], "HPG 1.32");
  
  // Get available firmware versions for ZED-F9R
  auto f9r_versions = loader_->get_available_firmware_versions("ZED-F9R");
  ASSERT_EQ(f9r_versions.size(), 3u);
  EXPECT_EQ(f9r_versions[0], "HPS 1.13");
  EXPECT_EQ(f9r_versions[1], "HPS 1.20");
  EXPECT_EQ(f9r_versions[2], "HPS 1.30");
  
  // Get available firmware versions for non-existent device
  auto non_existent = loader_->get_available_firmware_versions("ZED-F9T");
  EXPECT_EQ(non_existent.size(), 0u);
}

// Test error handling for non-existent file
TEST_F(UbxCfgParameterLoaderTest, NonExistentFile)
{
  // Create a loader with a non-existent file
  auto loader = std::make_unique<ubx::cfg::UbxCfgParameterLoader>("non_existent.json");
  
  // Attempt to load parameters
  EXPECT_THROW(loader->load(), ubx::cfg::ParameterLoadException);
}

// Test error handling for invalid JSON
TEST_F(UbxCfgParameterLoaderTest, InvalidJson)
{
  // Create an invalid JSON file
  auto invalid_file = test_dir_ / "invalid.json";
  std::ofstream file(invalid_file);
  file << "{ invalid json }";
  file.close();
  
  // Create a loader with the invalid file
  auto loader = std::make_unique<ubx::cfg::UbxCfgParameterLoader>(invalid_file.string());
  
  // Attempt to load parameters
  EXPECT_THROW(loader->load(), ubx::cfg::ParameterLoadException);
}

// Test error handling for missing required fields
TEST_F(UbxCfgParameterLoaderTest, MissingRequiredFields)
{
  // Create a JSON file with missing required fields
  auto missing_fields_file = test_dir_ / "missing_fields.json";
  std::ofstream file(missing_fields_file);
  file << R"({
    "version": "1.0.0"
  })";
  file.close();
  
  // Create a loader with the file
  auto loader = std::make_unique<ubx::cfg::UbxCfgParameterLoader>(missing_fields_file.string());
  
  // Attempt to load parameters
  EXPECT_THROW(loader->load(), ubx::cfg::ParameterLoadException);
}

}  // namespace

// Run all tests
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
