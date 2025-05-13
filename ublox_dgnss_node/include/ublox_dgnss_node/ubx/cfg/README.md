# UBX-CFG Parameter System Implementation

This directory contains the C++ implementation of the UBX-CFG parameter system with firmware version support.

## Overview

The UBX-CFG parameter system provides a flexible, data-driven approach to manage configuration parameters for u-blox GNSS receivers. The implementation consists of three main classes:

1. **UbxCfgParameter**: Represents individual parameters with all attributes
2. **UbxCfgParameterLoader**: Loads parameter definitions from JSON files
3. **UbxCfgHandler**: Manages parameter operations with the device

## Class Descriptions

### UbxCfgParameter

The `UbxCfgParameter` class encapsulates all the attributes of a UBX-CFG parameter, including:

- Basic properties (name, key ID, type, scale, unit)
- Applicable devices
- Description and group
- Firmware support information
- Value constraints (min/max values, possible values)

Key features:
- Firmware version comparison logic
- Value conversion between string and binary representations
- Value validation against constraints
- Behavior change tracking across firmware versions

```cpp
// Example usage
UbxCfgParameter parameter(
  "CFG_UART1_BAUDRATE",
  0x40520001,
  ubx::cfg::ubx_type_t::U4,
  1.0,
  ubx::cfg::ubx_unit_t::NA,
  {"ZED-F9P", "ZED-F9R"},
  "UART1 baud rate",
  "UART",
  firmware_support,  // Map of device to firmware support
  {},  // No possible values
  "38400",  // Default value
  "9600",   // Min value
  "921600"  // Max value
);

// Check firmware support
bool supported = parameter.is_supported_in_firmware("ZED-F9P", "HPG 1.30");

// Convert and validate values
ubx::value_t value = parameter.string_to_ubx_value("115200");
bool valid = parameter.validate_string_value("115200");
```

### UbxCfgParameterLoader

The `UbxCfgParameterLoader` class is responsible for loading parameter definitions from JSON files and filtering them based on device type and firmware version.

Key features:
- JSON parsing with error handling
- Parameter lookup by name and key ID
- Filtering by device type
- Filtering by device type and firmware version
- Firmware version information retrieval

```cpp
// Example usage
UbxCfgParameterLoader loader("config/ubx_cfg_parameters_full.json");

// Load parameters
bool success = loader.load();

// Get parameter by name
auto param = loader.get_parameter_by_name("CFG_UART1_BAUDRATE");

// Get parameters for a specific device
auto params = loader.get_parameters_for_device("ZED-F9P");

// Get parameters for a specific device and firmware version
auto filtered_params = loader.get_parameters_for_device_and_firmware(
  "ZED-F9P", "HPG 1.30"
);

// Get available firmware versions for a device
auto versions = loader.get_available_firmware_versions("ZED-F9P");
```

### UbxCfgHandler

The `UbxCfgHandler` class manages parameter operations, including detecting firmware versions and filtering parameters based on device type and firmware version.

Key features:
- Automatic firmware version detection
- Parameter registration with ROS
- Parameter change handling
- Parameter value validation
- Parameter behavior change tracking

```cpp
// Example usage
UbxCfgHandler handler(
  node,
  transceiver,
  std::make_shared<UbxCfgParameterLoader>("config/ubx_cfg_parameters_full.json"),
  "ZED-F9P"  // Default device type
);

// Initialize the handler
bool success = handler.initialize();

// Handle parameter changes
auto result = handler.on_parameter_change(parameters);

// Get detected firmware version
std::string version = handler.get_detected_firmware_version();

// Get parameter behavior change
std::string change = handler.get_parameter_behavior_change("CFG_RATE_MEAS");
```

## Integration with UbloxDGNSSNode

To use the UBX-CFG parameter system in the `UbloxDGNSSNode` class:

1. Include the necessary headers:
   ```cpp
   #include "ublox_dgnss_node/ubx/cfg/ubx_cfg_handler.hpp"
   ```

2. Add a member variable for the handler:
   ```cpp
   std::unique_ptr<ubx::cfg::UbxCfgHandler> cfg_handler_;
   ```

3. Initialize the handler in the constructor or initialization method:
   ```cpp
   cfg_handler_ = std::make_unique<ubx::cfg::UbxCfgHandler>(
     this,  // ROS node
     transceiver_,  // UBX transceiver
     std::make_shared<ubx::cfg::UbxCfgParameterLoader>(
       "config/ubx_cfg_parameters_full.json"
     ),
     "ZED-F9P"  // Default device type
   );
   
   cfg_handler_->initialize();
   ```

4. Set up a parameter callback:
   ```cpp
   // Set up parameter callback
   param_callback_handle_ = add_on_set_parameters_callback(
     [this](const std::vector<rclcpp::Parameter> & parameters) {
       return cfg_handler_->on_parameter_change(parameters);
     }
   );
   ```

## Unit Tests

The UBX-CFG parameter system includes comprehensive unit tests for all classes:

- **UbxCfgParameterTest**: Tests parameter properties, firmware support, and value validation
- **UbxCfgParameterLoaderTest**: Tests parameter loading, filtering, and error handling
- **UbxCfgHandlerTest**: Tests initialization, parameter changes, and firmware detection

To run the tests:

```bash
cd ublox_dgnss_fork
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select ublox_dgnss_node
```

## Future Enhancements

- Support for more device types
- More detailed firmware version compatibility information
- Parameter group-based filtering
- Parameter interdependency handling
- Parameter value serialization to configuration files
