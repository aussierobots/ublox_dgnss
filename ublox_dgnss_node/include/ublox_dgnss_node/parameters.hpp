// Copyright (c) 2025 Australian Robotics Supplies & Technology
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

#ifndef UBLOX_DGNSS_NODE__PARAMETERS_HPP_
#define UBLOX_DGNSS_NODE__PARAMETERS_HPP_

#include <map>
#include <string>
#include <mutex>
#include <memory>
#include <functional>
#include <optional>
#include <chrono>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "usb.hpp"
#include "ublox_dgnss_node/ubx/ubx_cfg_item.hpp"

namespace ublox_dgnss
{

// Type definitions for UBX configuration
using ubx_cfg_item_map_t = std::map<ubx::cfg::ubx_key_id_t, ubx::cfg::ubx_cfg_item_t>;

// Parameter state definitions
enum ParamValueSource
{
  UNKNOWN,          // Parameter exists in UBX cfg map but value unknown (std::nullopt)
  DEVICE_ACTUAL,    // Real value retrieved from u-blox device via CFG-VALGET
  START_ARG,        // From launch file/yaml/command args during initialization
  RUNTIME_USER      // From ros2 param set during operation
};

inline const char * to_string(ParamValueSource source)
{
  switch (source) {
    case ParamValueSource::UNKNOWN: return "UNKNOWN";
    case ParamValueSource::DEVICE_ACTUAL: return "DEVICE_ACTUAL";
    case ParamValueSource::START_ARG: return "START_ARG";
    case ParamValueSource::RUNTIME_USER: return "RUNTIME_USER";
    default: return "INVALID_SOURCE";
  }
}

// Parameter status for state machine (updated by USB device callbacks)
enum ParamStatus
{
  PARAM_INITIAL,    // Default value parameter used (not set by user or retrieved from GPS)
  PARAM_USER,       // Value set by user either overridden at startup or param set
  PARAM_LOADED,     // Loaded from GPS device - not all items have a value set
  PARAM_VALSET,     // Value sent to GPS device - might get rejected there
  PARAM_VALGET,     // Attempt to retrieve value from GPS device
  PARAM_ACKNAK      // Future version - poll for value or valset might not work
};

inline const char * to_string(ParamStatus status)
{
  switch (status) {
    case PARAM_INITIAL: return "PARAM_INITIAL";
    case PARAM_USER: return "PARAM_USER";
    case PARAM_LOADED: return "PARAM_LOADED";
    case PARAM_VALSET: return "PARAM_VALSET";
    case PARAM_VALGET: return "PARAM_VALGET";
    case PARAM_ACKNAK: return "PARAM_ACKNAK";
    default: return "INVALID_STATUS";
  }
}

// Parameter state structure
class ParamState
{
public:
  std::optional<rclcpp::ParameterValue> param_value;  // nullopt = unknown value
  ParamValueSource param_source;  // Where the value came from
  ParamStatus param_status;       // State machine status (updated by USB callbacks)
  bool needs_device_send;         // Simple flag for batching
  std::chrono::steady_clock::time_point last_modified;

  // Default constructor for map operations
  ParamState()
  : param_source(ParamValueSource::UNKNOWN), param_status(PARAM_INITIAL), needs_device_send(false),
    last_modified(std::chrono::steady_clock::now()) {}

  // C++17 compatible constructor
  ParamState(
    std::optional<rclcpp::ParameterValue> val, ParamValueSource src, ParamStatus stat,
    bool send, std::chrono::steady_clock::time_point modified)
  : param_value(val), param_source(src), param_status(stat), needs_device_send(send),
    last_modified(modified) {}

public:
  // === Builder pattern ===
  class Builder
  {
public:
    Builder()
    : value_(std::nullopt),
      source_(ParamValueSource::UNKNOWN),
      status_(PARAM_INITIAL),
      needs_device_send_(false),
      last_modified_(std::chrono::steady_clock::now()) {}

    Builder & with_value(const rclcpp::ParameterValue & val)
    {
      value_ = val;
      return *this;
    }

    Builder & with_value(std::optional<rclcpp::ParameterValue> val)
    {
      value_ = std::move(val);
      return *this;
    }

    Builder & with_source(ParamValueSource source)
    {
      source_ = source;
      return *this;
    }

    Builder & with_status(ParamStatus status)
    {
      status_ = status;
      return *this;
    }

    Builder & with_needs_device_send(bool send)
    {
      needs_device_send_ = send;
      return *this;
    }

    Builder & with_last_modified(std::chrono::steady_clock::time_point time)
    {
      last_modified_ = time;
      return *this;
    }

    ParamState build() const
    {
      // Validate that non-UNKNOWN sources have values
      if ((source_ != ParamValueSource::UNKNOWN) && !value_.has_value()) {
        throw std::logic_error("Builder: value must be set for known ParamValueSource");
      }

      // Validate that PARAM_USER and PARAM_VALSET have valid sources
      if ((status_ == PARAM_USER || status_ == PARAM_VALSET) &&
        source_ == ParamValueSource::UNKNOWN)
      {
        throw std::logic_error(
                "Builder: source must not be UNKNOWN for status PARAM_USER or PARAM_VALSET");
      }

      // Validate that PARAM_USER and PARAM_VALSET have values
      if ((status_ == PARAM_USER || status_ == PARAM_VALSET) && !value_.has_value()) {
        throw std::logic_error(
                "Builder: value must be set for status PARAM_USER or PARAM_VALSET");
      }

      // Validate that PARAM_INITIAL and PARAM_VALSET have values
      if (status_ == PARAM_INITIAL && value_.has_value()) {
        throw std::logic_error(
                "Builder: value must not be set for status PARAM_INITIAL");
      }

      // Validate needs_device_send consistency with status
      if (needs_device_send_ && (status_ != PARAM_USER && status_ != PARAM_INITIAL)) {
        throw std::logic_error(
                "Builder: needs_device_send should only be true for" \
                "PARAM_USER or PARAM_INITIAL status"
        );
      }

      // Validate timestamp is not in the future (allow 1 second tolerance)
      auto now = std::chrono::steady_clock::now();
      auto tolerance = std::chrono::seconds(1);
      if (last_modified_ > now + tolerance) {
        throw std::logic_error("Builder: last_modified timestamp cannot be in the future");
      }

      return ParamState(value_, source_, status_, needs_device_send_, last_modified_);
    }

private:
    std::optional<rclcpp::ParameterValue> value_;
    ParamValueSource source_;
    ParamStatus status_;
    bool needs_device_send_;
    std::chrono::steady_clock::time_point last_modified_;
  };
};


class ParameterException : public std::runtime_error
{
public:
  explicit ParameterException(std::string msg)
  : std::runtime_error(msg)
  {
  }
};

class ParameterStatusException : public std::runtime_error
{
public:
  explicit ParameterStatusException(std::string msg)
  : std::runtime_error(msg)
  {
  }
};

// Parameter manager class for centralized parameter handling with USB integration
class ParameterManager
{
public:
  // Constructor takes logger and USB connection for integration
  explicit ParameterManager(rclcpp::Logger logger);

  void set_device_batch_callback(std::function<bool(const std::vector<std::string> &)> callback);

  // Parameter cache management methods
  void initialize_ubx_config(const ubx_cfg_item_map_t & config_map);  // Load UBX config items

  void set_parameter_cache(
    const std::string & param_name, std::optional<rclcpp::ParameterValue>,
    ParamStatus initial_status,
    ParamValueSource source);
  void update_parameter_cache(
    const std::string & param_name, const rclcpp::ParameterValue & value,
    ParamStatus new_status,
    ParamValueSource source);

  std::optional<ParamState> get_parameter_state(const std::string & param_name);
  bool update_parameter_status(const std::string & param_name, ParamStatus p_status);

  // Parameter processing methods
  void process_parameter_cache();
  void restore_user_parameters_to_device();
  void reset_device_parameters();

  // UBX config item access (direct types)
  bool is_valid_parameter(const std::string & param_name);
  bool parameter_exists(const std::string & param_name);
  const ubx::cfg::ubx_cfg_item_t * find_config_item(const std::string & param_name);
  const ubx::cfg::ubx_cfg_item_t * find_config_item_by_key(const ubx::cfg::ubx_key_id_t & key_id);
  void load_config_items(const ubx_cfg_item_map_t & item_map);
  void iterate_config_items(std::function<void(const ubx::cfg::ubx_cfg_item_t &)> callback);

  void parameter_processing_callback();

  // Helper methods
  void log_parameter_cache_state();

private:
  // UBX config storage (direct - no PIMPL needed)
  std::map<std::string, ubx::cfg::ubx_cfg_item_t> config_items_;

  std::map<std::string, ParamState> param_cache_map_;
  std::mutex param_cache_mutex_;

  std::vector<std::string> pending_send_queue_;
  std::chrono::steady_clock::time_point last_parameter_change_;
  static constexpr size_t BATCH_SIZE_LIMIT = 10;

  // Logging
  rclcpp::Logger logger_;

  std::function<bool(const std::vector<std::string> &)> device_batch_callback_;
  void send_batch_parameters(
    const std::vector<std::string> & param_names,
    ParamStatus new_status = PARAM_VALSET);
  void send_parameter_to_device(
    const std::string & param_name,
    ParamStatus new_status = PARAM_VALSET);
};

}  // namespace ublox_dgnss

#endif  // UBLOX_DGNSS_NODE__PARAMETERS_HPP_
