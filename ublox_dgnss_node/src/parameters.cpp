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

#include "ublox_dgnss_node/parameters.hpp"

namespace ublox_dgnss
{

ParameterManager::ParameterManager(
  rclcpp::Logger logger)
: logger_(logger)
{
  RCLCPP_INFO(logger_, "ParameterManager initialized");
}


void ParameterManager::set_parameter_cache(
  const std::string & param_name,
  const std::optional<rclcpp::ParameterValue> value,
  ParamStatus initial_status,
  ParamValueSource source = ParamValueSource::RUNTIME_USER)
{
  std::lock_guard<std::mutex> lock(param_cache_mutex_);

  if (initial_status != PARAM_USER && initial_status != PARAM_INITIAL) {
    std::string msg = "Unable to set parameter cache: " + param_name +
      " to status: " + to_string(initial_status);
    RCLCPP_ERROR(logger_, msg.c_str());
    throw ParameterStatusException(msg);
  }

  RCLCPP_DEBUG(
    logger_, "Setting parameter cache: %s value: %s status: %s",
    param_name.c_str(),
    value ? rclcpp::Parameter(param_name, *value).value_to_string().c_str() : "<unset>",
    to_string(initial_status)
  );

  // ParamValueSource source = (initial_status == PARAM_USER) ?
  //   ParamValueSource::START_ARG :
  //   ParamValueSource::UNKNOWN;

  // bool needs_send = (initial_status == PARAM_USER);

  param_cache_map_[param_name] = ParamState::Builder()
    .with_value(value)
    .with_source(source)
    .with_status(initial_status)
    .with_needs_device_send(false)
    .with_last_modified(std::chrono::steady_clock::now())
    .build();
}

void ParameterManager::update_parameter_cache(
  const std::string & param_name,
  const rclcpp::ParameterValue & value,
  ParamStatus new_status,
  ParamValueSource source = ParamValueSource::RUNTIME_USER)
{
  std::lock_guard<std::mutex> lock(param_cache_mutex_);

  RCLCPP_DEBUG(
    logger_,
    "Updating parameter cache: %s value: %s status: %s",
    param_name.c_str(),
    rclcpp::Parameter(param_name, value).value_to_string().c_str(),
    to_string(new_status));

  const auto now = std::chrono::steady_clock::now();
  const bool needs_send = (new_status == PARAM_USER);

  auto it = param_cache_map_.find(param_name);
  if (it != param_cache_map_.end()) {
    it->second = ParamState::Builder()
      .with_value(value)
      .with_source(source)
      .with_status(new_status)
      .with_needs_device_send(needs_send)
      .with_last_modified(now)
      .build();
  } else {
    param_cache_map_[param_name] = ParamState::Builder()
      .with_value(value)
      .with_source(source)
      .with_status(new_status)
      .with_needs_device_send(needs_send)
      .with_last_modified(now)
      .build();
  }
}

void ParameterManager::set_device_batch_callback(
  std::function<bool(const std::vector<std::string> &)> callback)
{
  device_batch_callback_ = callback;
  RCLCPP_DEBUG(logger_, "Device batch callback set");
}

void ParameterManager::send_parameter_to_device(
  const std::string & param_name,
  ParamStatus new_status)
{
  RCLCPP_DEBUG(logger_, "Sending parameter: %s to device", param_name.c_str());

  if (device_batch_callback_) {
    try {
      bool success = device_batch_callback_({param_name});

      if (success) {
        std::lock_guard<std::mutex> lock(param_cache_mutex_);
        // Mark all parameters as sent in wall timer cache
        if (param_cache_map_.count(param_name)) {
          param_cache_map_[param_name].needs_device_send = false;
          param_cache_map_[param_name].param_status = new_status;  // Sent to device
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Failed to send parameter: %s - %s", param_name.c_str(), e.what());
    }
  } else {
    RCLCPP_WARN(logger_, "No device batch callback set - cannot send parameter to device");
  }
}

void ParameterManager::send_batch_parameters(
  const std::vector<std::string> & param_names,
  ParamStatus new_status)
{
  RCLCPP_DEBUG(logger_, "Sending batch of %zu parameters", param_names.size());

  if (device_batch_callback_) {
    try {
      bool success = device_batch_callback_(param_names);

      if (success) {
        std::lock_guard<std::mutex> lock(param_cache_mutex_);
        // Mark all parameters as sent in wall timer cache
        for (const auto & param_name : param_names) {
          if (param_cache_map_.count(param_name)) {
            param_cache_map_[param_name].needs_device_send = false;
            param_cache_map_[param_name].param_status = new_status;  // Sent to device
          }
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_, "Failed to send batch of %zu parameters - %s", param_names.size(),
        e.what());
    }
  } else {
    RCLCPP_WARN(logger_, "No device batch callback set - cannot send batch");
  }
}


void ParameterManager::initialize_ubx_config(const ubx_cfg_item_map_t & config_map)
{
  RCLCPP_DEBUG(logger_, "UBX config initialization requested");
  load_config_items(config_map);
}

void ParameterManager::load_config_items(const ubx_cfg_item_map_t & item_map)
{
  config_items_.clear();

  for (const auto & kv : item_map) {
    const auto & ubx_ci = kv.second;
    config_items_[ubx_ci.ubx_config_item] = ubx_ci;
  }

  RCLCPP_INFO(logger_, "Loaded %zu UBX config items", config_items_.size());
}

std::optional<ParamState> ParameterManager::get_parameter_state(const std::string & param_name)
{
  std::lock_guard<std::mutex> lock(param_cache_mutex_);

  auto it = param_cache_map_.find(param_name);
  if (it != param_cache_map_.end()) {
    return it->second;
  }

  // Return none
  return {};
}

bool ParameterManager::update_parameter_status(const std::string & param_name, ParamStatus p_status)
{
  std::lock_guard<std::mutex> lock(param_cache_mutex_);

  auto it = param_cache_map_.find(param_name);
  if (it != param_cache_map_.end()) {
    it->second.param_status = p_status;
    return true;
  }

  // Return
  return false;
}

void ParameterManager::restore_user_parameters_to_device()
{
  std::lock_guard<std::mutex> lock(param_cache_mutex_);

  size_t user_params_restored = 0;

  for (auto & [param_name, p_state] : param_cache_map_) {
    if (p_state.param_source == ParamValueSource::START_ARG ||
      p_state.param_source == ParamValueSource::RUNTIME_USER)
    {
      send_parameter_to_device(param_name, PARAM_VALSET);
      user_params_restored++;
    }
  }

  RCLCPP_INFO(logger_, "Restored %zu user parameters to device", user_params_restored);
}

void ParameterManager::reset_device_parameters()
{
  std::lock_guard<std::mutex> lock(param_cache_mutex_);

  size_t params_invalidated = 0;

  for (auto & [param_name, p_state] : param_cache_map_) {
    // Invalidate parameters that came from device but weren't overridden by user
    if (p_state.param_status != PARAM_USER) {
      RCLCPP_DEBUG(
        logger_,
        "Resetting device parameter: %s (was %s/%s)",
        param_name.c_str(),
        to_string(p_state.param_status),
        to_string(p_state.param_source));

      // Reset value and mark for refetch
      p_state.param_value = std::nullopt;
      p_state.param_source = ParamValueSource::UNKNOWN;
      p_state.param_status = PARAM_INITIAL;
      p_state.needs_device_send = false;

      param_cache_map_[param_name] = p_state;

      params_invalidated++;
    }
  }

  RCLCPP_WARN(logger_, "Reset %zu device parameters", params_invalidated);
}

void ParameterManager::parameter_processing_callback()
{
  RCLCPP_DEBUG(logger_, "Parameter processing callback triggered");

  // Collect parameters that need to be sent to device
  std::vector<std::string> params_to_send;

  {
    std::lock_guard<std::mutex> lock(param_cache_mutex_);

    for (auto & [param_name, param_state] : param_cache_map_) {
      if (param_state.needs_device_send && param_state.param_status == PARAM_USER) {
        params_to_send.push_back(param_name);
      }
    }
  }

  if (!params_to_send.empty()) {
    RCLCPP_DEBUG(
      logger_, "Parameter processing callback %zu user parameters for device",
      params_to_send.size());
    send_batch_parameters(params_to_send);
  }

  // Log current state for debugging
  log_parameter_cache_state();
}

void ParameterManager::log_parameter_cache_state()
{
  std::lock_guard<std::mutex> lock(param_cache_mutex_);

  size_t total_params = param_cache_map_.size();
  size_t initial_params = 0;
  size_t user_params = 0;
  size_t loaded_params = 0;
  size_t valset_params = 0;
  size_t valget_params = 0;
  size_t acknak_params = 0;

  for (const auto & [param_name, p_state] : param_cache_map_) {
    switch (p_state.param_status) {
      case PARAM_INITIAL:
        initial_params++;
        break;
      case PARAM_USER:
        user_params++;
        break;
      case PARAM_LOADED:
        loaded_params++;
        break;
      case PARAM_VALSET:
        valset_params++;
        break;
      case PARAM_VALGET:
        valget_params++;
        break;
      case PARAM_ACKNAK:
        acknak_params++;
        break;
      default:
        break;
    }
  }

  RCLCPP_DEBUG(
    logger_,
    "Parameter cache state - " \
    "Total: %zu, Initial: %zu, User: %zu, Loaded: %zu, Valset: %zu, Valget: %zu",
    total_params, initial_params, user_params, loaded_params, valset_params, valget_params);
}

bool ParameterManager::is_valid_parameter(const std::string & param_name)
{
  return config_items_.find(param_name) != config_items_.end();
}

bool ParameterManager::parameter_exists(const std::string & param_name)
{
  std::lock_guard<std::mutex> lock(param_cache_mutex_);

  return param_cache_map_.find(param_name) != param_cache_map_.end();
}

const ubx::cfg::ubx_cfg_item_t * ParameterManager::find_config_item(const std::string & param_name)
{
  auto it = config_items_.find(param_name);
  if (it != config_items_.end()) {
    return &it->second;
  }
  return nullptr;
}

const ubx::cfg::ubx_cfg_item_t * ParameterManager::find_config_item_by_key(
  const ubx::cfg::ubx_key_id_t & key_id)
{
  for (const auto & kv : config_items_) {
    if (kv.second.ubx_key_id.all == key_id.all) {
      return &kv.second;
    }
  }
  return nullptr;
}

void ParameterManager::iterate_config_items(
  std::function<void(const ubx::cfg::ubx_cfg_item_t &)> callback)
{
  for (const auto & kv : config_items_) {
    callback(kv.second);
  }
}

}  // namespace ublox_dgnss
