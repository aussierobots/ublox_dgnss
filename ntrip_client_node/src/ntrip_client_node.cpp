// Copyright 2023 Australian Robotics Supplies & Technology
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

#include <cstdio>
#include <curl/curl.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rtcm_msgs/msg/message.hpp"
#include "ntrip_client_node/visibility_control.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace ublox_dgnss
{
struct CurlHandle
{
  CURL * handle;
  CurlHandle()
  : handle(curl_easy_init()) {}
  ~CurlHandle() {curl_easy_cleanup(handle);}
};

class NTRIPClientNode : public rclcpp::Node
{
public:
  NTRIP_CLIENT_NODE_PUBLIC
  explicit NTRIPClientNode(const rclcpp::NodeOptions & options)
  : Node("ntrip_client",
      rclcpp::NodeOptions(options)),
    curlHandle_(std::make_shared<CurlHandle>())
  {
    RCLCPP_INFO(this->get_logger(), "starting %s", get_name());

    declare_parameter("use_https", true);
    declare_parameter("host", "ntrip.data.gnss.ga.gov.au");
    declare_parameter("port", 443);
    declare_parameter("mountpoint", "MBCH00AUS0");
    declare_parameter("username", "noname");
    declare_parameter("password", "password");

    use_https_ = get_parameter("use_https").as_bool();
    host_ = get_parameter("host").as_string();
    port_ = get_parameter("port").as_int();
    mountpoint_ = get_parameter("mountpoint").as_string();
    username_ = get_parameter("username").as_string();
    password_ = get_parameter("password").as_string();

    std::string url = ConnectionUrl();

    RCLCPP_INFO(this->get_logger(), "ntrip connection url: '%s'", url.c_str());

    std::string userpwd = username_ + ":" + password_;
    RCLCPP_DEBUG(this->get_logger(), "userpwd: '%s'", userpwd.c_str());

    // Create the publisher for rtcm_msgs::msg::Message
    rtcm_pub_ = this->create_publisher<rtcm_msgs::msg::Message>("/ntrip_client/rtcm", 10);

    curl_global_init(CURL_GLOBAL_DEFAULT);

    // Set the desired record count - libcurl will keep reading from the ntrip castor indefinitly
    // this is used to force it to exit in WriteCallback. In DoStreaming it will check to see if
    // streaing_exit_ is true, such that the ros2 node can terminate cleanly
    int desiredCount = 10;

    auto handle = curlHandle_->handle;
    if (handle) {
      curl_easy_setopt(handle, CURLOPT_URL, url.c_str());
      curl_easy_setopt(handle, CURLOPT_HTTP09_ALLOWED, true);
      curl_easy_setopt(handle, CURLOPT_USERPWD, userpwd.c_str());
      // curl_easy_setopt(handle, CURLOPT_NOPROGRESS, 0L);
      // curl_easy_setopt(handle, CURLOPT_VERBOSE, 1L);
      curl_easy_setopt(handle, CURLOPT_FAILONERROR, true);
      curl_easy_setopt(handle, CURLOPT_WRITEFUNCTION, &NTRIPClientNode::WriteCallback);
      curl_easy_setopt(curlHandle_->handle, CURLOPT_WRITEDATA, this);
      curl_easy_setopt(
        curlHandle_->handle, CURLOPT_PRIVATE,
        reinterpret_cast<void *>(desiredCount));

      // Start the streaming in a separate thread
      streaming_exit_ = false;
      streamingThread_ = std::thread(&NTRIPClientNode::DoStreaming, this);
    }
  }

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
  std::shared_ptr<CurlHandle> curlHandle_;
  std::thread streamingThread_;

  bool streaming_exit_;
  bool desired_count_reached_;

  // NTRIP castor connection
  bool use_https_;
  std::string host_;
  int port_;
  std::string mountpoint_;
  std::string username_;
  std::string password_;

  rclcpp::Publisher<rtcm_msgs::msg::Message>::SharedPtr rtcm_pub_;

  std::string ConnectionUrl()
  {
    std::string url;
    if (use_https_) {
      url = "https://" + host_ + ":" + std::to_string(port_) + "/" + mountpoint_;
    } else {
      url = "http://" + host_ + ":" + std::to_string(port_) + "/" + mountpoint_;
    }

    return url;
  }

  static size_t WriteCallback(char * ptr, size_t size, size_t nmemb, void * userdata)
  {
    NTRIPClientNode * node = reinterpret_cast<NTRIPClientNode *>(userdata);

    // code doesnt work in Humble
    // if (node->get_logger().get_effective_level() == rclcpp::Logger::Level::Debug) {
    // Convert the received data to a hexadecimal string
    std::stringstream hexStream;
    hexStream << std::hex << std::setfill('0');
    for (size_t i = 0; i < size * nmemb; i++) {
      hexStream << std::setw(2) << static_cast<int>(ptr[i]);
    }
    std::string hexString = hexStream.str();

    // Log the hexadecimal string as a debug message
    RCLCPP_DEBUG(
      node->get_logger(), "Received size: %ld nmemb: %ld data: %s", size, nmemb,
      hexString.c_str());
    // }

    // Create an instance of the message and populate
    auto message = std::make_unique<rtcm_msgs::msg::Message>();
    message->header.stamp = node->get_clock()->now();
    message->header.frame_id = node->mountpoint_;

    // Set the data from the char* ptr
    message->message.assign(ptr, ptr + size * nmemb);

    // Publish the message
    node->rtcm_pub_->publish(std::move(message));

    // Keep track of the number of records received
    static int recordCount = 0;
    recordCount++;

    // Get the desired count from the private parameter
    int desiredCount;
    curl_easy_getinfo(node->curlHandle_->handle, CURLINFO_PRIVATE, &desiredCount);


    // Check if the desired count is reached
    if (recordCount >= desiredCount) {
      recordCount = 0;

      node->desired_count_reached_ = true;

      // Returning a value different from the received data size
      // will signal libcurl to stop receiving further data
      return size * nmemb - 1;
    }

    // Returning the actual received data size will continue the stream
    return size * nmemb;
  }

  void DoStreaming()
  {
    while (!streaming_exit_) {
      desired_count_reached_ = false;

      // Perform the request
      CURLcode res = curl_easy_perform(curlHandle_->handle);

      // Check for any errors
      if (res != CURLE_OK) {

        if (desired_count_reached_) {
          RCLCPP_DEBUG(this->get_logger(), "Processed desired count... ");
          // Sleep for 100 mili seconds
          rclcpp::sleep_for(std::chrono::milliseconds(100));
        } else {

          // Retrieve and log the effective URL
          char * effectiveUrl;
          curl_easy_getinfo(curlHandle_->handle, CURLINFO_EFFECTIVE_URL, &effectiveUrl);
          RCLCPP_ERROR(
            this->get_logger(), "Failed to perform streaming request for URL: %s", effectiveUrl);

          // Retrieve and log the response code
          long responseCode;
          curl_easy_getinfo(curlHandle_->handle, CURLINFO_RESPONSE_CODE, &responseCode);
          RCLCPP_ERROR(this->get_logger(), "Response code: %ld", responseCode);

          // Handle the error
          RCLCPP_ERROR(
            this->get_logger(), "Failed to perform streaming request: %s", curl_easy_strerror(res));

          // Sleep for 1 second
          rclcpp::sleep_for(std::chrono::seconds(1));
        }
      }
    }
  }

public:
  NTRIP_CLIENT_NODE_LOCAL
  ~NTRIPClientNode()
  {
    streaming_exit_ = true;

    // Wait for the streaming thread to finish
    streamingThread_.join();

    curlHandle_.reset();
    curl_global_cleanup();
    RCLCPP_INFO(this->get_logger(), "finished");
  }
};
}  // namespace ublox_dgnss

RCLCPP_COMPONENTS_REGISTER_NODE(ublox_dgnss::NTRIPClientNode)
