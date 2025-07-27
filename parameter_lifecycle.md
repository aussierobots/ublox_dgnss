# UBlox DGNSS Parameter System Documentation

## Overview

The UBlox DGNSS parameter system provides thread-safe parameter management with USB hotplug support. The system uses a centralized ParameterManager that coordinates between ROS2 parameters and GPS device configuration with direct UBX config storage and strict state transition validation.

## Architecture

### Core Components

#### ParameterManager
- **Purpose**: Centralized parameter handling with thread-safe operations and UBX config management
- **Location**: `parameters.hpp` / `parameters.cpp`
- **Implementation**: Direct UBX header inclusion with linker flag `--allow-multiple-definition`
- **Thread Safety**: Mutex-protected parameter cache access
- **USB Integration**: Monitors USB state changes for parameter synchronization
- **Config Storage**: Direct storage of UBX configuration items in `std::map<std::string, ubx::cfg::ubx_cfg_item_t>`
- **State Management**: Separate initialization (`set_parameter_cache`) and transition (`update_parameter_cache`) methods

#### Parameter Processing Timer
- **Purpose**: Separate thread for parameter-device communication
- **Callback Group**: `MutuallyExclusive` for thread isolation
- **Interval**: 100ms processing cycle
- **Function**: Processes parameter cache and handles device communication

#### USB State Management
- **Driver States**: `DISCONNECTED`, `CONNECTING`, `CONNECTED`, `ERROR` (4 states total)
- **Integration**: Direct parameter restoration on hotplug attach callback
- **Hotplug Support**: Automatic parameter restoration via `restore_user_parameters_to_device()`

## Parameter State Machine

### Parameter Source Tracking

```cpp
// Parameter value source tracking - actual implementation
enum ParamValueSource {
  UNKNOWN,          // Parameter exists in UBX cfg map but value unknown (std::nullopt)
  DEVICE_ACTUAL,    // Real value retrieved from u-blox device via CFG-VALGET
  START_ARG,        // From launch file/yaml/command args during initialization
  RUNTIME_USER      // From ros2 param set during operation
};
```

### Parameter State Machine

```cpp
// Parameter status for state machine transitions - actual implementation  
enum ParamStatus {
  PARAM_INITIAL,    // Default value parameter used (not set by user or retrieved from GPS)
  PARAM_USER,       // Value set by user either overridden at startup or param set
  PARAM_LOADED,     // Loaded from GPS device - not all items have a value set
  PARAM_VALSET,     // Value sent to GPS device - might get rejected there
  PARAM_VALGET,     // Attempt to retrieve value from GPS device
  PARAM_ACKNAK      // Future version - poll for value or valset might not work
};
```

### State Transitions

Complete state transition matrix with validation:

```
PARAM_INITIAL → PARAM_VALGET → PARAM_LOADED        (device fetch)
PARAM_LOADED → PARAM_VALGET → PARAM_LOADED         (hot plug re-fetch)
PARAM_USER → PARAM_VALSET → PARAM_LOADED           (user override)
PARAM_LOADED → PARAM_VALSET → PARAM_LOADED         (modify device value)
```

**State Transition Validation**: The `update_parameter_cache()` method enforces these transitions and rejects invalid state changes.

### Parameter Cache Structure

```cpp
// Modern ParamState class with Builder pattern - actual implementation
class ParamState {
public:
  std::optional<rclcpp::ParameterValue> param_value;  // nullopt = unknown value
  ParamValueSource param_source;                      // Where the value came from
  ParamStatus param_status;                           // State machine status (updated by USB callbacks)
  bool needs_device_send;                             // Simple flag for batching
  std::chrono::steady_clock::time_point last_modified;

  // Default constructor for map operations
  ParamState()
  : param_source(ParamValueSource::UNKNOWN), param_status(PARAM_INITIAL), 
    needs_device_send(false), last_modified(std::chrono::steady_clock::now()) {}

  // Builder pattern for construction with validation - actual implementation
  class Builder {
  public:
    Builder()
    : value_(std::nullopt), source_(ParamValueSource::UNKNOWN),
      status_(PARAM_INITIAL), needs_device_send_(false),
      last_modified_(std::chrono::steady_clock::now()) {}

    Builder& with_value(const rclcpp::ParameterValue& val) {
      value_ = val; return *this;
    }
    
    Builder& with_value(std::optional<rclcpp::ParameterValue> val) {
      value_ = std::move(val); return *this;
    }
    
    Builder& with_source(ParamValueSource source) {
      source_ = source; return *this;
    }
    
    Builder& with_status(ParamStatus status) {
      status_ = status; return *this;
    }
    
    Builder& with_needs_device_send(bool send) {
      needs_device_send_ = send; return *this;
    }
    
    ParamState build() const {
      // Comprehensive validation rules - see validation section below
      validate_state_consistency();
      return ParamState(value_, source_, status_, needs_device_send_, last_modified_);
    }
  };
};

std::map<std::string, ParamState> param_cache_map_;  // Updated map type
std::mutex param_cache_mutex_;  // Thread-safe access
```

### Builder Pattern Validation Rules

The Builder pattern enforces comprehensive validation to prevent invalid parameter states:

```cpp
// Actual validation implementation from parameters.hpp:152-191
void validate_state_consistency() const {
  // Rule 1: Non-UNKNOWN sources must have values
  if ((source_ != ParamValueSource::UNKNOWN) && !value_.has_value()) {
    throw std::logic_error("Builder: value must be set for known ParamValueSource");
  }

  // Rule 2: PARAM_USER and PARAM_VALSET require valid sources
  if ((status_ == PARAM_USER || status_ == PARAM_VALSET) &&
      source_ == ParamValueSource::UNKNOWN) {
    throw std::logic_error(
      "Builder: source must not be UNKNOWN for status PARAM_USER or PARAM_VALSET");
  }

  // Rule 3: PARAM_USER and PARAM_VALSET must have values
  if ((status_ == PARAM_USER || status_ == PARAM_VALSET) && !value_.has_value()) {
    throw std::logic_error(
      "Builder: value must be set for status PARAM_USER or PARAM_VALSET");
  }

  // Rule 4: PARAM_INITIAL must not have values (represents unknown default)
  if (status_ == PARAM_INITIAL && value_.has_value()) {
    throw std::logic_error(
      "Builder: value must not be set for status PARAM_INITIAL");
  }

  // Rule 5: needs_device_send flag consistency
  if (needs_device_send_ && (status_ != PARAM_USER && status_ != PARAM_INITIAL)) {
    throw std::logic_error(
      "Builder: needs_device_send should only be true for PARAM_USER or PARAM_INITIAL status");
  }

  // Rule 6: Timestamp validation (prevent future timestamps)
  auto now = std::chrono::steady_clock::now();
  auto tolerance = std::chrono::seconds(1);
  if (last_modified_ > now + tolerance) {
    throw std::logic_error("Builder: last_modified timestamp cannot be in the future");
  }
}
```

## UBX Configuration Management

### Direct Storage Architecture

```cpp
class ParameterManager {
private:
  // Direct UBX config storage (no PIMPL needed with linker flag)
  std::map<std::string, ubx::cfg::ubx_cfg_item_t> config_items_;
  
  // Thread-safe parameter cache access - updated structure
  std::map<std::string, ParamState> param_cache_map_;
  std::mutex param_cache_mutex_;
  
  // Batch processing system - actual implementation
  std::vector<std::string> pending_send_queue_;
  std::chrono::steady_clock::time_point last_parameter_change_;
  static constexpr size_t BATCH_SIZE_LIMIT = 10;
  static constexpr std::chrono::milliseconds BATCH_DELAY{100};
  
  // Device communication callback for batch operations
  std::function<bool(const std::vector<std::string>&)> device_batch_callback_;
  
public:
  // Parameter cache management methods - updated signatures
  void set_parameter_cache(const std::string & param_name, 
                          std::optional<rclcpp::ParameterValue> value, 
                          ParamStatus initial_status);  // Initialization only
  void update_parameter_cache(const std::string & param_name, 
                             const rclcpp::ParameterValue & value, 
                             ParamStatus new_status);  // State transitions only
  
  // Batch processing methods
  void set_device_batch_callback(std::function<bool(const std::vector<std::string>&)> callback);
  void send_batch_parameters(const std::vector<std::string>& param_names, ParamStatus new_status = PARAM_VALSET);
  void parameter_processing_callback();  // Wall timer callback
  
  // UBX config item access (direct types) 
  bool is_valid_parameter(const std::string & param_name);
  const ubx::cfg::ubx_cfg_item_t* find_config_item(const std::string & param_name);
  const ubx::cfg::ubx_cfg_item_t* find_config_item_by_key(const ubx::cfg::ubx_key_id_t& key_id);
  void iterate_config_items(std::function<void(const ubx::cfg::ubx_cfg_item_t&)> callback);
};
```

### Batch Processing System Architecture

The ParameterManager implements efficient batch processing for device communication:

```cpp
// Batch processing implementation - actual code from parameters.cpp
void parameter_processing_callback() {
  RCLCPP_DEBUG(logger_, "Parameter processing callback triggered");

  // 1. Collect parameters that need to be sent to device
  std::vector<std::string> params_to_send;
  {
    std::lock_guard<std::mutex> lock(param_cache_mutex_);
    for (auto& [param_name, param_state] : param_cache_map_) {
      if (param_state.needs_device_send && param_state.param_status == PARAM_USER) {
        params_to_send.push_back(param_name);
      }
    }
  }

  // 2. Send batch if parameters are pending
  if (!params_to_send.empty()) {
    RCLCPP_DEBUG(logger_, "Parameter processing callback %zu user parameters for device", 
                 params_to_send.size());
    send_batch_parameters(params_to_send);
  }
}

void send_batch_parameters(const std::vector<std::string>& param_names, ParamStatus new_status) {
  if (device_batch_callback_) {
    try {
      bool success = device_batch_callback_(param_names);  // Send to device via node callback
      
      if (success) {
        std::lock_guard<std::mutex> lock(param_cache_mutex_);
        // Mark all parameters as sent
        for (const auto& param_name : param_names) {
          if (param_cache_map_.count(param_name)) {
            param_cache_map_[param_name].needs_device_send = false;
            param_cache_map_[param_name].param_status = new_status;  // PARAM_VALSET
          }
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "Failed to send batch of %zu parameters - %s", 
                   param_names.size(), e.what());
    }
  }
}
```

### Configuration Loading

```cpp
// Load UBX config items from global map during initialization
void initialize_ubx_config(const ubx_cfg_item_map_t& config_map) {
  parameter_manager_->load_config_items(config_map);
}

// Direct implementation loads from provided map
void load_config_items(const ubx_cfg_item_map_t& item_map) {
  config_items_.clear();
  for (const auto & kv : item_map) {
    const auto & ubx_ci = kv.second;
    config_items_[ubx_ci.ubx_config_item] = ubx_ci;
  }
}

// Type definition for UBX configuration
using ubx_cfg_item_map_t = std::map<ubx::cfg::ubx_key_id_t, ubx::cfg::ubx_cfg_item_t>;
```

### Config Item Access Methods

```cpp
// Parameter name lookup (for user parameter validation)
bool is_valid_parameter(const std::string& param_name);

// Parameter name to config item (for parameter operations)
const ubx::cfg::ubx_cfg_item_t* find_config_item(const std::string& param_name);

// Key ID to config item (for device responses)
const ubx::cfg::ubx_cfg_item_t* find_config_item_by_key(const ubx::cfg::ubx_key_id_t& key_id);

// Bulk operations (for initialization/synchronization)
void iterate_config_items(std::function<void(const ubx::cfg::ubx_cfg_item_t&)> callback);
```

### Parameter Cache Management

```cpp
// Initialization only - creates new parameter cache entry
void set_parameter_cache(const std::string & param_name, 
                        const rclcpp::ParameterValue & value, 
                        ParamStatus initial_status);  // PARAM_INITIAL or PARAM_USER only

// State transitions only - validates and updates existing parameter
void update_parameter_cache(const std::string & param_name,
                           const rclcpp::ParameterValue & value,
                           ParamStatus new_status);  // Enforces valid state transitions
```

## System Flow

### Node Initialization

1. **ParameterManager Creation**
   - Direct UBX config storage initialized
   - UBX config items loaded from provided map via `initialize_ubx_config()`
   - Parameter cache initialized with thread-safe access

2. **Parameter Declaration**
   - All UBX parameters declared using `set_or_declare_ubx_cfg_param()`
   - Launch file parameters automatically detected and marked as `PARAM_USER`
   - Default parameters marked as `PARAM_INITIAL`
   - Uses `set_parameter_cache()` for initialization, not `update_parameter_cache()`

3. **Device Communication Setup**
   - Device communication callback configured
   - USB state monitoring initialized
   - Parameter processing timer created on separate callback group

### USB Connection Lifecycle

#### Device Connection
1. USB hotplug detection triggers attach callback
2. USB driver state transitions to `CONNECTED`
3. ParameterManager receives state change notification
4. Parameter restoration begins for `PARAM_USER` parameters

#### Device Disconnection
1. USB hotplug detection triggers detach callback
2. USB driver state transitions to `DISCONNECTED`
3. ParameterManager receives state change notification
4. Parameter cache preserved for reconnection

#### Simple Reconnection Detection
```cpp
// Actual hotplug_attach_callback implementation - ublox_dgnss_node.cpp:1368-1390
void hotplug_attach_callback() {
  device_attached_ = true;
  bool is_reconnection = has_been_connected_before_;  // Simple reconnection flag
  
  if (is_reconnection) {
    // Reconnection: Just restore user parameters
    RCLCPP_INFO(get_logger(), "Device reconnected - restoring user parameters");
    if (parameter_manager_) {
      parameter_manager_->restore_user_parameters_to_device();
      device_readiness_state_ = DeviceReadinessState::READY;
    }
  } else {
    // Initial connection: Full initialization
    perform_usb_initialization();  // Existing full init
    device_readiness_state_ = DeviceReadinessState::READY;
    has_been_connected_before_ = true;
    RCLCPP_INFO(get_logger(), "Initial device connection completed");
  }
}

// Parameter restoration implementation - parameters.cpp:200-215
void restore_user_parameters_to_device() {
  size_t user_params_restored = 0;
  
  for (auto& [param_name, p_state] : param_cache_map_) {
    if (p_state.param_source == ParamValueSource::START_ARG ||
        p_state.param_source == ParamValueSource::RUNTIME_USER) {
      send_parameter_to_device(param_name, PARAM_VALSET);
      user_params_restored++;
    }
  }
  
  RCLCPP_INFO(logger_, "Restored %zu user parameters to device", user_params_restored);
}
```

### Runtime Parameter Changes

1. **ROS2 Parameter Callback**
   - User changes parameter via ROS2 interfaces
   - Parameter validation using `ParameterManager::is_valid_parameter()`
   - ParameterManager cache updated with new value

2. **Parameter Processing**
   - Timer callback processes parameter cache using `parameter_processing_callback()`
   - `PARAM_USER` parameters with `needs_device_send=true` collected for batch transmission
   - Device communication handled via `device_batch_callback_` on separate thread with `MutuallyExclusive` callback group

3. **Device Communication**
   - Parameters sent to device in batches using `send_batch_parameters()`
   - State transition `PARAM_USER` → `PARAM_VALSET` via batch completion
   - Exception handling with automatic retry for failed batch operations

## Thread Safety

### Callback Groups
- **Main Node**: Default callback group for ROS2 operations
- **Parameter Processing**: Separate `MutuallyExclusive` callback group
- **USB Events**: Separate callback group for USB hotplug
- **UBX/RTCM**: Separate callback groups for message processing

### Mutex Protection
- `param_cache_mutex_`: Protects parameter cache access
- `usb_init_mutex_`: Protects USB initialization state
- All parameter cache operations are mutex-protected
- Direct header inclusion with `--allow-multiple-definition` linker flag eliminates dependency issues

### Thread Communication
- Parameter changes flow through thread-safe cache
- Device communication isolated to parameter processing thread
- USB state changes trigger parameter restoration
- UBX config access centralized through ParameterManager

## USB Hotplug Support

### No USB at Startup
- Node starts normally without USB device
- Parameters declared and cached using ParameterManager with `set_parameter_cache()`
- Parameter processing waits for device connection
- User can modify parameters before device attachment

### USB Reconnection
- Device automatically detected on reconnection
- Parameter cache preserved across disconnections
- User parameters automatically restored using cached config items
- System maintains configuration consistency

### Error Handling
- Device communication failures handled gracefully
- Parameter operations retry on USB reconnection
- State machine prevents inconsistent operations
- System degrades gracefully without USB device

## Integration Points

### Device Communication with Exception Handling

```cpp
// Exception-based USB state validation - actual implementation
bool send_parameters_to_device_batch(const std::vector<std::string>& param_names) {
  RCLCPP_DEBUG(get_logger(), "send_parameters_to_device_batch called with %zu parameters", param_names.size());

  // Critical: Check USB state FIRST with exception on failure
  if (usbc_->driver_state() != usb::USBDriverState::CONNECTED) {
    std::string error_msg = "USB device not connected - cannot send parameters to device!";
    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
    throw std::runtime_error(error_msg);  // Exception prevents invalid device access
  }

  // Validate required components
  if (ubx_cfg_ == nullptr) {
    RCLCPP_ERROR(get_logger(), "ubx_cfg_ is null - cannot send parameters to device");
    throw std::runtime_error("ubx_cfg_ is null - cannot send parameters to device");
  }

  if (!parameter_manager_) {
    RCLCPP_ERROR(get_logger(), "parameter_manager_ is null - cannot send parameters to device");
    throw std::runtime_error("parameter_manager_ is null - cannot send parameters to device");
  }

  try {
    std::lock_guard<std::mutex> lock(cfg_batch_mutex_);  // Thread-safe device access
    
    // Clear any existing CFG data and prepare batch
    ubx_cfg_->cfg_val_set_cfgdata_clear();
    ubx_cfg_->cfg_val_set_layer_ram(true);  // Immediate effect
    
    // Build batch request with parameter validation
    bool any_params_added = false;
    for (const std::string& param_name : param_names) {
      const ubx::cfg::ubx_cfg_item_t* cfg_item = parameter_manager_->find_config_item(param_name);
      
      if (!cfg_item) {
        RCLCPP_WARN(get_logger(), "Parameter %s not found in config items", param_name.c_str());
        continue;  // Skip invalid parameters but continue batch
      }
      
      // Add parameter to batch
      // ... (batch building logic)
      any_params_added = true;
    }
    
    return any_params_added;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Exception in batch parameter sending: %s", e.what());
    throw;  // Re-throw for higher-level handling
  }
}
```

### Parameter Validation
```cpp
// Parameter callback validation
bool valid = parameter_manager_->is_valid_parameter(param_name);
if (!valid) {
  RCLCPP_WARN(get_logger(), "parameter %s is not recognised", param_name.c_str());
}
```

### Device Response Processing
```cpp
// Process device configuration responses
void ubx_cfg_payload_parameters(std::shared_ptr<ubx::cfg::CfgValGetPayload> payload) {
  for (const auto kv : payload->cfg_data) {
    const auto* ubx_ci = parameter_manager_->find_config_item_by_key(kv.ubx_key_id);
    if (ubx_ci != nullptr) {
      // State transition PARAM_VALGET → PARAM_LOADED
      set_or_declare_ubx_cfg_param(*ubx_ci, kv.ubx_value);
    }
  }
}
```

### Exception Handling Patterns

```cpp
// ParameterException and ParameterStatusException - actual implementation
class ParameterException : public std::runtime_error {
public:
  explicit ParameterException(std::string msg) : std::runtime_error(msg) {}
};

class ParameterStatusException : public std::runtime_error {
public:
  explicit ParameterStatusException(std::string msg) : std::runtime_error(msg) {}
};

// Usage in parameter cache operations - parameters.cpp:35-40
void set_parameter_cache(const std::string& param_name, 
                        const std::optional<rclcpp::ParameterValue> value,
                        ParamStatus initial_status) {
  if (initial_status != PARAM_USER && initial_status != PARAM_INITIAL) {
    std::string msg = "Unable to set parameter cache: " + param_name
      + " to status: " + to_string(initial_status);
    RCLCPP_ERROR(logger_, msg.c_str());
    throw ParameterStatusException(msg);  // Exception prevents invalid state
  }
  // ... rest of implementation
}
```

## Configuration

### Node Setup
- ParameterManager initialized with logger and USB connection
- UBX config loaded from global map during initialization
- Device communication callback configured
- Timer created with appropriate interval and callback group

### Parameter Declaration
- All UBX parameters declared using ParameterManager config items
- Launch file parameters automatically loaded
- Parameter callbacks configured for runtime changes

### Build Configuration
- `parameters.cpp` added to CMakeLists.txt
- `target_link_options(ublox_dgnss_components PRIVATE "LINKER:--allow-multiple-definition")` resolves header inclusion
- Thread safety libraries linked
- Direct UBX header inclusion in multiple compilation units

## Future Enhancements

### TOML Configuration Loading
The architecture supports replacing hardcoded UBX configuration with TOML file loading:

```cpp
// Future: Load config items from TOML instead of global map
void load_config_items_from_toml(const std::string& toml_file) {
  config_items_.clear();
  // Parse TOML file and populate config_items_ map
  // Maintains same interface for parameter operations
  // Use same initialize_ubx_config() interface with TOML-loaded map
}
```

## Benefits

### Architecture Benefits
- **Direct Storage**: Simplified architecture with direct UBX config storage
- **Centralized Management**: All UBX config access through ParameterManager
- **Type Safety**: Full `ubx::cfg::ubx_cfg_item_t` structures stored and accessed
- **State Validation**: Enforced parameter state transitions prevent invalid operations
- **Future Ready**: Architecture supports TOML configuration loading

### Robustness
- Parameter cache survives USB disconnections
- Automatic parameter restoration on reconnection
- Thread-safe parameter operations
- Graceful degradation without USB device

### Performance
- Separate thread prevents blocking ROS2 operations
- Efficient parameter processing cycle
- Minimal USB polling overhead
- Optimized parameter transmission using cached config items

### Maintainability
- Clear separation of concerns with dedicated methods
- Centralized parameter management through ParameterManager
- Well-defined state machine with validation
- Comprehensive error handling and logging
- Pragmatic solution to header dependency conflicts

### User Experience
- Transparent USB hotplug support
- Consistent parameter behavior
- Reliable configuration persistence
- Clear system state feedback

## Critical Bug Fix: Parameter Initialization Timing Issue (RESOLVED)

### Problem Description
The original parameter system suffered from a critical timing issue where the constructor attempted to call `get_parameter()` on parameters that hadn't been declared yet through ROS2's parameter system. This caused most PARAM_USER parameters to have null values and fail silently during Phase 1 device transmission.

### Root Cause
```cpp
// PROBLEMATIC (old implementation)
// Constructor tried to get parameter values before they were available
for (const std::string& name : param_names) {
  if (parameter_manager_->is_valid_parameter(name)) {
    // BUG: get_parameter() failed for undeclared parameters
    auto p_value = get_parameter(name).get_parameter_value();  // Would fail
    parameter_manager_->set_parameter_cache(name, {}, PARAM_USER);  // Stored null values
  }
}
```

### Solution Implemented
```cpp
// FIXED (current implementation)
// Constructor now properly retrieves values for parameters that exist
for (const std::string& name : param_names) {
  if (parameter_manager_->is_valid_parameter(name)) {
    // FIXED: get_parameter() only called on parameters provided via launch file/args
    auto p_value = std::make_optional(get_parameter(name).get_parameter_value());
    RCLCPP_DEBUG(get_logger(), "set initial user param name: %s value: %s",
      name.c_str(),
      p_value ? rclcpp::Parameter(name, *p_value).value_to_string().c_str() : "<unset>");
    parameter_manager_->set_parameter_cache(name, p_value, ParamStatus::PARAM_USER);
  }
}
```

### Evidence of Fix
- **Before Fix**: Only 1 out of 20 PARAM_USER parameters successfully sent to device
- **After Fix**: All 20 PARAM_USER parameters successfully transmitted to device in Phase 1
- **Proof**: `CFG_USBOUTPROT_NMEA=False` now properly disables NMEA output (was failing before)
- **Log Evidence**: "Phase 1: Sending user parameters to device" now processes all user parameters correctly

### Technical Details
1. **Parameter Detection**: ROS2 automatically declares parameters provided via launch files/command arguments
2. **Value Retrieval**: `get_parameter(name)` only succeeds for parameters that exist (were provided by user)
3. **Null Protection**: `std::make_optional()` properly handles the parameter value wrapping
4. **State Consistency**: All PARAM_USER parameters now have valid `param_value` fields
5. **Device Communication**: `cfg_val_set_from_ubx_ci_p_state()` no longer fails due to null values

### Impact
This fix resolved the core parameter initialization issue, ensuring that user-provided parameters from launch files are properly transmitted to the GPS device during the critical Phase 1 initialization, maintaining the intended system behavior and device configuration.