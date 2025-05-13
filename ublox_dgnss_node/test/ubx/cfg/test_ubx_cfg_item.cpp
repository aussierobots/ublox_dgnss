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
 * @file test_ubx_cfg_item.cpp
 * @brief Unit tests for the UbxCfgItem structure
 */

#include <gtest/gtest.h>
#include <string>

#include "ublox_dgnss_node/ubx/cfg/ubx_cfg_item.hpp"
#include "ublox_dgnss_node/ubx/ubx_types.hpp"

namespace ubx {
namespace cfg {
namespace test {

/**
 * @brief Test fixture for UbxCfgItem
 */
class UbxCfgItemTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a test item with all required fields
    item_.name = "CFG-RATE-MEAS";
    item_.key_id = 0x30210001;
    item_.type = ubx::cfg::U1; // Use the correct enum from ubx::cfg namespace
    item_.scale = 1.0;
    item_.unit = ubx::cfg::MS; // Using the correct enum value (capital MS)
  }

  ubx_cfg_item_t item_;
};

/**
 * @brief Test basic properties of a UbxCfgItem
 */
TEST_F(UbxCfgItemTest, BasicProperties)
{
  EXPECT_EQ(item_.name, "CFG-RATE-MEAS");
  EXPECT_EQ(item_.key_id, 0x30210001);
  EXPECT_EQ(item_.type, ubx::cfg::U1);
  EXPECT_DOUBLE_EQ(item_.scale, 1.0);
  EXPECT_EQ(item_.unit, ubx::cfg::MS);
}

/**
 * @brief Test key_id extraction methods
 */
TEST_F(UbxCfgItemTest, KeyIdExtraction)
{
  // Create a ubx_key_id_t from the integer key_id
  ubx_key_id_t key_id;
  key_id.all = item_.key_id;

  // Use the methods of ubx_key_id_t to extract components
  EXPECT_EQ(key_id.group_id(), 0x21);
  EXPECT_EQ(key_id.item_id(), 0x0001);
  EXPECT_EQ(key_id.storage_size_id(), 3); // 3 corresponds to U2 (16-bit)
}

/**
 * @brief Test edge case with maximum values
 */
TEST_F(UbxCfgItemTest, EdgeCaseMaxValues)
{
  // Create an item with maximum values for testing
  ubx_cfg_item_t max_item;
  max_item.name = "CFG-MAX-TEST";
  max_item.key_id = 0xFFFFFFFF;
  max_item.type = ubx::cfg::X8;
  max_item.scale = 255.5;
  max_item.unit = ubx::cfg::NA;

  // Create a ubx_key_id_t from the key_id
  ubx_key_id_t key_id;
  key_id.all = max_item.key_id;

  // Test the extraction methods
  EXPECT_EQ(key_id.group_id(), 0xFF);
  EXPECT_EQ(key_id.item_id(), 0xFFF); // Only the 12 bits are used
  EXPECT_EQ(key_id.storage_size_id(), 7); // 7 corresponds to X8 (64-bit)
}

/**
 * @brief Test key_id string representation
 */
TEST_F(UbxCfgItemTest, KeyIdStringRepresentation)
{
  ubx_key_id_t key_id;
  key_id.all = 0x30210001;
  
  // Test the to_hex method
  EXPECT_EQ(key_id.to_hex(), "0x30210001");
}

/**
 * @brief Test key_id comparison operators
 */
TEST_F(UbxCfgItemTest, KeyIdComparison)
{
  ubx_key_id_t key_id1, key_id2;
  key_id1.all = 0x30210001;
  key_id2.all = 0x30210002;
  
  // Test operator<
  EXPECT_TRUE(key_id1 < key_id2);
  EXPECT_FALSE(key_id2 < key_id1);
  
  // Test operator==
  EXPECT_TRUE(key_id1 == 0x30210001);
  EXPECT_TRUE(0x30210001 == key_id1);
  EXPECT_FALSE(key_id1 == key_id2);
}

}  // namespace test
}  // namespace cfg
}  // namespace ubx

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
