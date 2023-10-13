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

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "ublox_nav_sat_fix_hp_node/visibility_control.h"
#include "ublox_ubx_msgs/msg/gps_fix.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_cov.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_hp_pos_llh.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_status.hpp"

using std::placeholders::_1;

// size of position covariance array
static const size_t POS_COV_ARR_SIZE = 9;

namespace ublox_nav_sat_fix_hp
{

class UbloxNavSatHpFixNode : public rclcpp::Node
{
public:
  UBLOX_NAV_SAT_FIX_HP_NODE_PUBLIC
  explicit UbloxNavSatHpFixNode(const rclcpp::NodeOptions & options)
  : Node("ublox_nav_sat_fix_hp",
      rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
  {
    RCLCPP_INFO(this->get_logger(), "starting %s", get_name());

    enu_pos_cov_.fill(0.0);  // initialise values to zero

    auto qos = rclcpp::SensorDataQoS();

    // Create publishers
    nav_sat_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", qos);

    // Create subscribers
    ubx_nav_hp_pos_llh_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavHPPosLLH>(
      "ubx_nav_hp_pos_llh", qos,
      std::bind(&UbloxNavSatHpFixNode::nav_hp_pos_llh_callback, this, std::placeholders::_1));
    ubx_nav_cov_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavCov>(
      "ubx_nav_cov", qos,
      std::bind(&UbloxNavSatHpFixNode::nav_cov_callback, this, std::placeholders::_1));
    ubx_nav_status_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavStatus>(
      "ubx_nav_status", qos,
      std::bind(&UbloxNavSatHpFixNode::nav_sta_callback, this, std::placeholders::_1));
  }

  UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
  ~UbloxNavSatHpFixNode() {RCLCPP_INFO(this->get_logger(), "finished");}

private:
  // void nav_hp_pos_llh_callback(const ublox_ubx_msgs::msg::UBXNavHPPosLLH::SharedPtr llh_msg);
  // void nav_cov_callback(const ublox_ubx_msgs::msg::UBXNavCov::SharedPtr nav_cov_msg);
  // void nav_sta_callback(const ublox_ubx_msgs::msg::UBXNavStatus::SharedPtr nav_sta_msg);

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_pub_;

  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavHPPosLLH>::SharedPtr ubx_nav_hp_pos_llh_sub_;
  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavCov>::SharedPtr ubx_nav_cov_sub_;
  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavStatus>::SharedPtr ubx_nav_status_sub_;

  // std::vector<double> enu_covariance_diagonal_;
  std::array<double, POS_COV_ARR_SIZE> enu_pos_cov_;
  sensor_msgs::msg::NavSatStatus nav_sat_stat_;

  // flags used to check whether we have received corresponding messages
  bool have_recd_enu_pos_cov_ = false;

  UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
  void nav_hp_pos_llh_callback(
    const ublox_ubx_msgs::msg::UBXNavHPPosLLH::SharedPtr ubx_hppos_llh_msg)
  {
    // Create the NavSatFix message
    sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
    // header - copy from Pos message
    nav_sat_fix_msg.header = ubx_hppos_llh_msg->header;
    // copy status from previous nav_sat_stat message
    nav_sat_fix_msg.status = nav_sat_stat_;

    // Extract the LLH and high-precision components
    double lat = ubx_hppos_llh_msg->lat * 1e-7 + ubx_hppos_llh_msg->lat_hp * 1e-9;
    double lon = ubx_hppos_llh_msg->lon * 1e-7 + ubx_hppos_llh_msg->lon_hp * 1e-9;
    double alt = ubx_hppos_llh_msg->height * 1e-3 + ubx_hppos_llh_msg->height_hp * 1e-4;

    // Convert the LLH position and covariance values to NavSatFix message format
    nav_sat_fix_msg.latitude = lat;   // Degrees
    nav_sat_fix_msg.longitude = lon;  // Degrees
    nav_sat_fix_msg.altitude = alt;   // meters

    // Fill in covariance data
    if (nav_sat_fix_msg.position_covariance.size() != enu_pos_cov_.size()) {
      RCLCPP_ERROR(
        this->get_logger(), "Size mismatch betwwen NavSatFix covariance data and EnuPosCov data");
      return;
    }
    for (size_t i = 0; i < enu_pos_cov_.size(); i++) {
      nav_sat_fix_msg.position_covariance[i] = enu_pos_cov_[i];
    }
    if (have_recd_enu_pos_cov_) {
      // Set covariance type to estimated from the converted NED to ENU covariance
      nav_sat_fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
    } else {
      nav_sat_fix_msg.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }

    // Publish NavSatFix message
    nav_sat_fix_pub_->publish(nav_sat_fix_msg);

    RCLCPP_DEBUG(
      this->get_logger(), "Published NavSatFix with lat %4f lon %4f alt %4f", lat, lon, alt);
  }

  UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
  void nav_cov_callback(const ublox_ubx_msgs::msg::UBXNavCov::SharedPtr ubx_cov_msg)
  {
    // 6 position covariance values available in UBX-NAV-COV matrix
    // Matrix is symmetrix, so only upper triangular values are shown
    // pos_cov_nn
    // pos_cov_ne
    // pos_cov_nd
    // pos_cov_ee
    // pos_cov_ed
    // pos_cov_dd

    // In matrix notation, the values in NED coordinate system are
    // C_NED = | Pnn Pne Pnd |
    //         | Pne Pee Ped |
    //         | Pnd Ped Pdd |

    // After transformation into ENU coordinate system, the matrix becomes
    // C_ENU = | Pee  Pne -Ped |
    //         | Pne  Pnn -Pnd |
    //         |-Ped -Pnd  Pdd |

    // Tranform the covariance matrix from NED to ENU format in row-major order
    static_assert(POS_COV_ARR_SIZE == 9, "size of enu_pos_cov_ must be 9");
    enu_pos_cov_[0] = ubx_cov_msg->pos_cov_ee;
    enu_pos_cov_[1] = ubx_cov_msg->pos_cov_ne;
    enu_pos_cov_[2] = -ubx_cov_msg->pos_cov_ed;
    enu_pos_cov_[3] = ubx_cov_msg->pos_cov_ne;
    enu_pos_cov_[4] = ubx_cov_msg->pos_cov_nn;
    enu_pos_cov_[5] = -ubx_cov_msg->pos_cov_nd;
    enu_pos_cov_[6] = -ubx_cov_msg->pos_cov_ed;
    enu_pos_cov_[7] = -ubx_cov_msg->pos_cov_nd;
    enu_pos_cov_[8] = ubx_cov_msg->pos_cov_dd;

    // set flag to show we have received fresh data for this message
    have_recd_enu_pos_cov_ = true;
  }

  UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
  void nav_sta_callback(const ublox_ubx_msgs::msg::UBXNavStatus::SharedPtr ubx_sta_msg)
  {
    // UBX NAV STATUS values do not map very cleanly to ROS2 sensor_msgs/msg/NavSatStatus values.
    // Do the best we can to indicate whether we have GPS fix or not
    switch (ubx_sta_msg->gps_fix.fix_type) {
      case ublox_ubx_msgs::msg::GpsFix::GPS_NO_FIX:
      case ublox_ubx_msgs::msg::GpsFix::GPS_TIME_ONLY:
      case ublox_ubx_msgs::msg::GpsFix::GPS_DEAD_RECKONING_ONLY:
        nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        break;
      case ublox_ubx_msgs::msg::GpsFix::GPS_FIX_2D:
      case ublox_ubx_msgs::msg::GpsFix::GPS_FIX_3D:
      case ublox_ubx_msgs::msg::GpsFix::GPS_PLUS_DEAD_RECKONING:
        if (true == ubx_sta_msg->diff_soln) {  // diff corrections were applied
          nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        } else {
          nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        }
        break;
      default:
        nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        break;
    }

    // Service values - derive from UBX-NAV-SAT gnssId field?
    // In their absence, use arrogant default assumption of GPS
    nav_sat_stat_.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  }
};

}  // namespace ublox_nav_sat_fix_hp

RCLCPP_COMPONENTS_REGISTER_NODE(ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode)
