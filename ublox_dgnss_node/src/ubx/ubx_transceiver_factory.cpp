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
 * @file ubx_transceiver_factory.cpp
 * @brief Implementation of the UbxTransceiverFactory class
 */

#include "ublox_dgnss_node/ubx/ubx_transceiver_factory.hpp"
#include "ublox_dgnss_node/ubx/usb_ubx_transceiver.hpp"
#include "ublox_dgnss_node/ubx/mock_ubx_transceiver.hpp"

namespace ubx
{

std::shared_ptr<UbxTransceiver> UbxTransceiverFactory::create_usb_transceiver(
  std::shared_ptr<usb::Connection> usbc,
  rclcpp::Logger logger)
{
  return std::make_shared<UsbUbxTransceiver>(usbc, logger);
}

std::shared_ptr<UbxTransceiver> UbxTransceiverFactory::create_usb_transceiver(
  rclcpp::Node * node,
  std::shared_ptr<usb::Connection> usbc)
{
  return std::make_shared<UsbUbxTransceiver>(usbc, node->get_logger());
}

std::shared_ptr<UbxTransceiver> UbxTransceiverFactory::create_mock_transceiver()
{
  return std::make_shared<MockUbxTransceiver>();
}

}  // namespace ubx
