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
 * @file ubx_transceiver_factory.hpp
 * @brief Factory for creating UbxTransceiver instances
 */

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_TRANSCEIVER_FACTORY_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_TRANSCEIVER_FACTORY_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ublox_dgnss_node/ubx/ubx_transceiver.hpp"
#include "ublox_dgnss_node/usb.hpp"

namespace ubx
{

/**
 * @brief Factory class for creating UbxTransceiver instances
 */
class UbxTransceiverFactory
{
public:
  /**
   * @brief Create a USB-based UbxTransceiver
   * @param usbc USB connection
   * @param logger ROS logger
   * @return UbxTransceiver instance
   */
  static std::shared_ptr<UbxTransceiver> create_usb_transceiver(
    std::shared_ptr<usb::Connection> usbc,
    rclcpp::Logger logger);

  /**
   * @brief Create a USB-based UbxTransceiver from a Node
   * @param node ROS node
   * @param usbc USB connection
   * @return UbxTransceiver instance
   */
  static std::shared_ptr<UbxTransceiver> create_usb_transceiver(
    rclcpp::Node * node,
    std::shared_ptr<usb::Connection> usbc);

#ifdef UBLOX_DGNSS_TESTING
  /**
   * @brief Create a mock UbxTransceiver for testing
   * @return UbxTransceiver instance
   */
  static std::shared_ptr<UbxTransceiver> create_mock_transceiver();
#endif // UBLOX_DGNSS_TESTING
};

}  // namespace ubx

#endif  // UBLOX_DGNSS_NODE__UBX__UBX_TRANSCEIVER_FACTORY_HPP_
