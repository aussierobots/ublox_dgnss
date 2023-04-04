#ifndef UBLOX_NAVSATFIX_NODE_HPP_
#define UBLOX_NAVSATFIX_NODE_HPP_

// #include <memory>
// #include <vector>
// #include <string>

#include "rclcpp/rclcpp.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_hp_pos_llh.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_cov.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_status.hpp"
#include "ublox_ubx_msgs/msg/gps_fix.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

class UbloxNavSatFix : public rclcpp::Node
{
public:
    UbloxNavSatFix();

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

#endif  // UBLOX_NAVSATFIX_NODE_HPP_