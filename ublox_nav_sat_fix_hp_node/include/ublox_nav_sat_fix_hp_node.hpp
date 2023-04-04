// Copyright 2023 CMP Engineers Pty Ltd
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

#ifndef UBLOX_NAV_SAT_FIX_HP_NODE_HPP_
#define UBLOX_NAV_SAT_FIX_HP_NODE_HPP_

// #include <memory>
// #include <vector>
// #include <string>

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "ublox_ubx_msgs/msg/gps_fix.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_cov.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_hp_pos_llh.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_status.hpp"

class UbloxNavSatHpFix : public rclcpp::Node
{
public:
  UbloxNavSatHpFix();

private:
  void nav_hp_pos_llh_callback(const ublox_ubx_msgs::msg::UBXNavHPPosLLH::SharedPtr llh_msg);
  void nav_cov_callback(const ublox_ubx_msgs::msg::UBXNavCov::SharedPtr nav_cov_msg);
  void nav_sta_callback(const ublox_ubx_msgs::msg::UBXNavStatus::SharedPtr nav_sta_msg);

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_pub_;

  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavHPPosLLH>::SharedPtr ubx_nav_hp_pos_llh_sub_;
  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavCov>::SharedPtr ubx_nav_cov_sub_;
  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavStatus>::SharedPtr ubx_nav_status_sub_;

  // std::vector<double> enu_covariance_diagonal_;
  std::vector<double> enu_pos_cov_;
  sensor_msgs::msg::NavSatStatus nav_sat_stat_;
};

#endif  // UBLOX_NAV_SAT_FIX_HP_NODE_HPP_
