# UBlox DGNSS USB Device State Management

## Overview

The UBlox DGNSS USB device state management system provides robust hotplug support with automatic device detection, connection handling, and state synchronization. The system uses a multi-layered state machine to track USB connection status and coordinate device operations across the application.

## Architecture

### Core Components

#### USB Connection Class
- **Purpose**: Low-level USB device communication and hotplug detection
- **Location**: `usb.hpp` / `usb.cpp`
- **Responsibilities**: libusb integration, device enumeration, transfer management
- **State Tracking**: Device attachment status and driver operational state

#### USB Driver State Machine
- **Purpose**: High-level USB connection state management
- **Integration**: Coordinates with ParameterManager and node operations
- **States**: Tracks connection lifecycle from detection to operational status
- **Notifications**: Provides state change callbacks to dependent systems

#### Hotplug Detection
- **Technology**: libusb hotplug callbacks
- **Events**: `DEVICE_ARRIVED` and `DEVICE_LEFT` notifications
- **Filtering**: Vendor/Product ID matching for UBlox devices
- **Callbacks**: Asynchronous notification system

## USB Driver State Machine

### State Definitions

```cpp
enum class USBDriverState {
  DISCONNECTED,     // No USB device present
  CONNECTING,       // Device detected, attempting connection
  CONNECTED,        // USB device connected and accessible
  ERROR             // Error state requiring recovery
};
```

### State Transitions

```
DISCONNECTED → CONNECTING (device detected via hotplug)
CONNECTING → CONNECTED (device opened successfully)
CONNECTING → ERROR (device open failed)
CONNECTED → DISCONNECTED (device unplugged)
ERROR → CONNECTING (recovery attempt via timer)
```

### State Machine Logic

```cpp
// USB state tracking - actual implementation in usb.cpp
class Connection {
private:
  USBDriverState driver_state_ = USBDriverState::DISCONNECTED;
  bool attached_ = false;
  
public:
  USBDriverState driver_state() const { return driver_state_; }
  bool attached() const { return attached_; }
  
  // State transitions occur in:
  bool open_device() {
    driver_state_ = USBDriverState::CONNECTING;
    // ... device opening logic ...
    if (success) {
      // State set to CONNECTED in hotplug_attach_callback
      return true;
    } else {
      driver_state_ = USBDriverState::ERROR;
      return false;
    }
  }
};
```

## Connection Lifecycle

### Device Detection
1. **Hotplug Event**: libusb detects device arrival via `LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED`
2. **Device Matching**: Vendor/Product ID verification (F9_VENDOR_ID/F9_PRODUCT_ID)
3. **State Transition**: `DISCONNECTED` → `CONNECTING`
4. **Connection Attempt**: Device enumeration and interface claiming via `open_device()`

### Device Connection
1. **Device Opening**: Claim USB interfaces and endpoints (CDC-ACM device handling)
2. **Interface Configuration**: Detach kernel drivers and claim interfaces
3. **State Transition**: `CONNECTING` → `CONNECTED` (set in hotplug_attach_callback)
4. **Node Notification**: Trigger node-level hotplug_attach_callback

### Device Disconnection  
1. **Hotplug Event**: libusb detects device removal via `LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT`
2. **Resource Cleanup**: Release interfaces and handles via `close_devh()`
3. **State Transition**: `CONNECTED` → `DISCONNECTED`
4. **Node Notification**: Trigger node-level hotplug_detach_callback

## Implementation Details

### USB Connection Management

```cpp
class Connection {
private:
  USBDriverState driver_state_;
  bool attached_;
  libusb_device_handle* devh_;
  
public:
  bool open_device();
  void close_device();
  USBDriverState get_driver_state() const;
  void set_driver_state(USBDriverState state);
};
```

### State Tracking Variables

```cpp
// Node-level device readiness state - actual implementation in ublox_dgnss_node.cpp
enum class DeviceReadinessState {
  UNREADY,    // Device not operational (no parameters sent to device)
  READY       // Device operational (parameters synchronized)
};

DeviceReadinessState device_readiness_state_ = DeviceReadinessState::UNREADY;
bool device_attached_ = false;
bool has_been_connected_before_ = false;   // Track reconnection vs initial connection
```

### Hotplug Callbacks

```cpp
// Low-level USB hotplug detection
int hotplug_attach_callback(libusb_context* ctx, libusb_device* dev, 
                           libusb_hotplug_event event, void* user_data) {
  if (!attached_) {
    if (open_device()) {
      attached_ = true;
      driver_state_ = USBDriverState::CONNECTED;
      // Notify node-level handler
    }
  }
}

int hotplug_detach_callback(libusb_context* ctx, libusb_device* dev,
                           libusb_hotplug_event event, void* user_data) {
  if (attached_) {
    close_device();
    attached_ = false;
    driver_state_ = USBDriverState::DISCONNECTED;
    // Notify node-level handler
  }
}
```

### Node-Level Integration

```cpp
// High-level device state management - actual implementation
void hotplug_attach_callback() {
  device_attached_ = true;
  bool is_reconnection = has_been_connected_before_;
  
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

void hotplug_detach_callback() {
  RCLCPP_WARN(get_logger(), "USB device disconnected");
  device_attached_ = false;
  device_readiness_state_ = DeviceReadinessState::UNREADY;
}
```

## Timer-Based Operations

### USB Initialization Timer
- **Purpose**: Periodic device detection when no device present
- **Interval**: 10ms polling cycle
- **Lifecycle**: Active when `usb_driver_state_ == NOT_STARTED`
- **Cancellation**: Disabled when device detected

### USB Events Timer
- **Purpose**: Process libusb events asynchronously
- **Interval**: 10ms processing cycle
- **Callback Group**: Separate `MutuallyExclusive` group
- **Function**: Handle USB transfers and hotplug events

```cpp
// Timer initialization
usb_init_timer_ = create_wall_timer(
  10ms, std::bind(&UbloxDGNSSNode::handle_usb_init_callback, this));

handle_usb_events_timer_ = create_wall_timer(
  10ms, std::bind(&UbloxDGNSSNode::handle_usb_events_callback, this),
  callback_group_usb_events_timer_);
```

## Thread Safety

### Mutex Protection
- **usb_init_mutex_**: Protects USB initialization state
- **Critical Sections**: Device attachment/detachment operations
- **Lock Scope**: Minimal to prevent blocking hotplug callbacks

### Callback Groups
- **USB Events**: Separate callback group for USB event processing
- **Hotplug Callbacks**: Executed on libusb thread context
- **Node Callbacks**: Executed on node's callback executor

### Asynchronous Operations
- **Hotplug Detection**: Asynchronous libusb callbacks
- **Device Initialization**: Non-blocking initialization sequence
- **Event Processing**: Periodic timer-based event handling

## Error Handling

### Connection Failures
- **Detection**: Failed device open operations
- **Response**: Transition to `ERROR` state
- **Recovery**: Automatic retry through timer mechanism
- **Logging**: Detailed error information for debugging

### Device Removal
- **Detection**: Hotplug detach events
- **Response**: Graceful resource cleanup
- **State Preservation**: Maintain configuration for reconnection
- **Notification**: Inform dependent systems of disconnection

### Initialization Failures
- **Detection**: Failed device configuration
- **Response**: Transition to `ERROR` state
- **Recovery**: Reset to `NOT_STARTED` for retry
- **Fallback**: Graceful degradation without USB device

## Integration Points

### Parameter System Integration
```cpp
// Parameter restoration on device reconnection - actual implementation
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

### Device Communication
```cpp
// USB state checking in actual implementation
bool send_parameters_to_device_batch(const std::vector<std::string>& param_names) {
  if (usbc_->driver_state() != usb::USBDriverState::CONNECTED) {
    std::string error_msg = "USB device not connected - cannot send parameters to device!";
    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  }
  // ... parameter sending logic ...
}
```

### State Queries
```cpp
// Actual method implementations
USBDriverState driver_state() const {    // Note: method name is driver_state(), not get_driver_state()
  return driver_state_;
}

bool attached() const {                  // Note: method name is attached(), not is_device_attached()
  return attached_;
}

// Node-level state queries
bool is_device_ready() const {
  return device_readiness_state_ == DeviceReadinessState::READY;
}
```

## Configuration

### USB Device Matching
- **Vendor ID**: UBlox-specific vendor identification
- **Product ID**: Device-specific product identification
- **Serial String**: Optional device-specific serial matching
- **Class ID**: `LIBUSB_HOTPLUG_MATCH_ANY` for flexibility

### Hotplug Registration
```cpp
// Register hotplug callbacks
libusb_hotplug_register_callback(
  ctx_, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED, 
  LIBUSB_HOTPLUG_ENUMERATE, vendor_id_, product_id_, class_id_,
  hotplug_attach_callback_fn, NULL, &hp_[0]);

libusb_hotplug_register_callback(
  ctx_, LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
  LIBUSB_HOTPLUG_ENUMERATE, vendor_id_, product_id_, class_id_,
  hotplug_detach_callback_fn, NULL, &hp_[1]);
```

## Benefits

### Robustness
- **Automatic Recovery**: Self-healing on device reconnection
- **Error Resilience**: Graceful handling of connection failures
- **State Consistency**: Coordinated state management across components
- **Resource Management**: Proper cleanup on disconnection

### Performance
- **Efficient Polling**: Minimal overhead when device present
- **Asynchronous Operations**: Non-blocking device operations
- **Event-Driven**: Responsive to device state changes
- **Optimized Timers**: Appropriate intervals for different operations

### Reliability
- **Comprehensive Testing**: Handles all device connection scenarios
- **State Machine**: Prevents invalid state transitions
- **Mutex Protection**: Thread-safe operations
- **Error Logging**: Detailed diagnostics for troubleshooting

### User Experience
- **Transparent Hotplug**: Seamless device connection/disconnection
- **No Manual Intervention**: Automatic device detection and setup
- **Consistent Behavior**: Predictable system response to device events
- **Status Feedback**: Clear indication of device connection state

## Enhanced Parameter Integration (NEW)

### DeviceReadinessState and ParameterManager Coordination

The USB state management now works closely with the ParameterManager to ensure that device readiness properly reflects parameter synchronization:

```cpp
// Enhanced hotplug callbacks with parameter integration - actual implementation
void hotplug_attach_callback() {
  bool is_reconnection = has_been_connected_before_;
  device_attached_ = true;
  
  if (is_reconnection) {
    // Reconnection: Just restore user parameters
    RCLCPP_INFO(get_logger(), "Device reconnected - restoring user parameters");
    if (parameter_manager_) {
      parameter_manager_->restore_user_parameters_to_device();
      device_readiness_state_ = DeviceReadinessState::READY;  // Ready when params restored
    }
  } else {
    // Initial connection: Full initialization
    perform_usb_initialization();  // This includes 3-phase parameter init
    device_readiness_state_ = DeviceReadinessState::READY;   // Ready when init complete
    has_been_connected_before_ = true;
    RCLCPP_INFO(get_logger(), "Initial device connection completed");
  }
}

void hotplug_detach_callback() {
  RCLCPP_WARN(get_logger(), "USB device disconnected");
  device_attached_ = false;
  device_readiness_state_ = DeviceReadinessState::UNREADY;  // Not ready when disconnected
}
```

### Key Improvements

1. **Parameter-Aware State Management**: `DeviceReadinessState` now transitions to `READY` only after parameters are synchronized
2. **Reconnection Optimization**: On reconnection, only user parameters are restored (not full initialization)
3. **State Consistency**: Device readiness precisely matches parameter synchronization status
4. **Initialization Integration**: Full 3-phase parameter initialization integrated with initial USB connection

### Benefits

- **Accurate State Tracking**: Device readiness now guarantees parameter synchronization
- **Efficient Reconnection**: Fast parameter restoration without full re-initialization
- **System Consistency**: USB and parameter states remain synchronized across all operations
- **Reliable Operation**: Device operations only permitted when parameters are properly configured