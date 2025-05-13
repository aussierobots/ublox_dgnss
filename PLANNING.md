# UBX-CFG Configuration System Refactoring

## Project Overview

This document outlines the plan for refactoring the UBX-CFG configuration system in the ublox_dgnss driver to support a more maintainable and extensible approach to handling configuration parameters for both ZED-F9P and ZED-F9R devices.

## Reference Documents

This implementation is based on the following u-blox documents:

- ZED-F9P Interface Description (UBX-22008968)
- ZED-F9R Interface Description (UBX-22010984)

These documents can be obtained from the u-blox website:
https://www.u-blox.com/en/product-resources

## Current Implementation Details

Based on code analysis, the current implementation defines and handles UBX-CFG parameters in the following way:

### Parameter Definition (ubx_cfg_item.hpp)
- Parameters are defined as `ubx_cfg_item_t` structures with hardcoded attributes:
  ```cpp
  const ubx_cfg_item_t CFG_INFMSG_UBX_USB = {"CFG_INFMSG_UBX_USB", 0x20920004, X1, 1, NA};
  ```
- Each parameter has:
  - Name (string identifier)
  - Key ID (u4_t - 32-bit unsigned integer)
  - Type (ubx_type_t - enumeration of data types like X1, L, U1, etc.)
  - Scale (double_t - scaling factor for the value)
  - Unit (ubx_unit_t - enumeration of units like NA, M, S, etc.)
- All parameters are registered in the `ubxKeyCfgItemMap` map for lookup

### Parameter Handling (ubx_cfg.hpp)
- Contains classes for UBX-CFG protocol messages:
  - `CfgValGetPayload`: For retrieving parameter values from the device
  - `CfgValSetPayload`: For setting parameter values on the device
- Provides utility functions for converting between UBX values and binary representations

### Node Implementation (ublox_dgnss_node.cpp)
- Parameters are checked against the `ubxKeyCfgItemMap` during initialization
- Parameter values are converted between ROS parameter types and UBX types
- Parameter changes trigger UBX-CFG-VALSET messages to the device
- Parameter values are retrieved from the device using UBX-CFG-VALGET messages

### Key Limitations
1. **Tight Coupling**: Parameter definitions are tightly coupled with code
2. **Maintenance Burden**: Adding or modifying parameters requires code changes and recompilation
3. **Limited Documentation**: Parameter descriptions and possible values are not well documented
4. **No Device Filtering**: No easy way to filter parameters by device type
5. **Scalability Issues**: As the number of parameters grows, the codebase becomes harder to maintain

## Proposed Solution

We will implement a data-driven approach using JSON configuration files with the following components:

### 1. Parameter Definition File (JSON)

The JSON schema will include:

```json
{
  "version": "1.0.0",
  "device_types": ["ZED-F9P", "ZED-F9R"],
  "parameters": [
    {
      "name": "CFG_INFMSG_UBX_USB",
      "key_id": "0x20920004",
      "type": "X1",
      "scale": 1.0,
      "unit": "NA",
      "applicable_devices": ["ZED-F9P", "ZED-F9R"],
      "description": "Information message configuration for UBX protocol on USB",
      "possible_values": {
        "INFMSG_ERROR": "0x01",
        "INFMSG_WARNING": "0x02",
        "INFMSG_NOTICE": "0x04",
        "INFMSG_TEST": "0x08",
        "INFMSG_DEBUG": "0x10"
      },
      "default_value": "0x00"
    }
    // ... more parameters
  ]
}
```

### 2. C++ Implementation

#### UbxCfgParameter Class
```cpp
class UbxCfgParameter {
public:
  // Constructor with firmware support
  UbxCfgParameter(
    const std::string & name,
    const ubx_key_id_t & key_id,
    const ubx_type_t & type,
    double scale,
    const ubx_unit_t & unit,
    const std::vector<std::string> & applicable_devices,
    const std::string & description,
    const std::string & group,
    const std::map<std::string, FirmwareSupport> & firmware_support,
    const std::map<std::string, std::string> & possible_values,
    const std::string & default_value,
    const std::string & min_value = "",
    const std::string & max_value = ""
  );

  // Getters for all properties
  std::string get_name() const;
  ubx_key_id_t get_key_id() const;
  ubx_type_t get_type() const;
  double get_scale() const;
  ubx_unit_t get_unit() const;
  std::vector<std::string> get_applicable_devices() const;
  std::string get_description() const;
  std::string get_group() const;
  std::map<std::string, FirmwareSupport> get_firmware_support() const;
  std::map<std::string, std::string> get_possible_values() const;
  std::string get_default_value() const;
  std::optional<std::string> get_min_value() const;
  std::optional<std::string> get_max_value() const;

  // Firmware version support methods
  bool is_applicable_to_device(const std::string & device_type) const;
  bool is_supported_in_firmware(const std::string & device_type, const std::string & firmware_version) const;
  std::string get_behavior_change(const std::string & device_type, const std::string & firmware_version) const;
  static int compare_firmware_versions(const std::string & version1, const std::string & version2);

  // Value conversion and validation
  ubx::value_t string_to_ubx_value(const std::string & value_str) const;
  std::string ubx_value_to_string(const ubx::value_t & value) const;
  bool validate_string_value(const std::string & value_str) const;

private:
  std::string name_;
  ubx_key_id_t key_id_;
  ubx_type_t type_;
  double scale_;
  ubx_unit_t unit_;
  std::vector<std::string> applicable_devices_;
  std::string description_;
  std::string group_;
  std::map<std::string, FirmwareSupport> firmware_support_;
  std::map<std::string, std::string> possible_values_;
  std::string default_value_;
  std::optional<std::string> min_value_;
  std::optional<std::string> max_value_;
  std::string default_value;
  
  // Utility methods for value conversion, validation, etc.
};
```

#### UbxCfgParameterLoader Class
```cpp
class UbxCfgParameterLoader {
public:
  UbxCfgParameterLoader(const std::string& file_path);
  
  // Load parameters from JSON file
  bool load();
  
  // Get parameter by name
  std::optional<UbxCfgParameter> get_parameter_by_name(const std::string& name);
  
  // Get parameter by key ID
private:
  std::string file_path_;
  std::vector<UbxCfgParameter> parameters_;
  std::map<std::string, UbxCfgParameter> parameters_by_name_;
  std::map<uint32_t, UbxCfgParameter> parameters_by_key_id_;
  std::vector<std::string> device_types_;
  std::map<std::string, std::vector<std::string>> firmware_versions_;
};

#### UbxCfgHandler Class
```cpp
class UbxCfgHandler {
public:
  // Constructor with node, transceiver, parameter loader, and device type
  UbxCfgHandler(
    rclcpp::Node * node,
    std::shared_ptr<UbxTransceiver> transceiver,
    std::shared_ptr<UbxCfgParameterLoader> parameter_loader,
    const std::string & device_type
  );

  // Initialize the handler
  bool initialize();

  // Parameter change callback for ROS
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter>& parameters);
  
  // Get parameter value from device
  bool get_parameter_value(const std::string& name);
  
  // Set parameter value on device
  bool set_parameter_value(const std::string& name, const std::string& value);

  // Firmware version detection
  std::string get_detected_firmware_version() const;
  void process_mon_ver(
    const std::string & sw_version,
    const std::string & hw_version,
    const std::vector<std::string> & extensions
  );

  // Parameter behavior change tracking
  std::string get_parameter_behavior_change(const std::string & parameter_name) const;

private:
  rclcpp::Node* node_;
  std::shared_ptr<UbxTransceiver> transceiver_;
  std::shared_ptr<UbxCfgParameterLoader> parameter_loader_;
  std::string device_type_;
  std::string firmware_version_;
  std::map<std::string, UbxCfgParameter> parameters_;
  std::map<std::string, std::string> parameter_behavior_changes_;
};

### Implementation Approach

The implementation of the UBX-CFG parameter system with firmware version support follows these key design principles:

1. **Data-Driven Design**: Parameter definitions are stored in JSON files rather than hardcoded in the source code, making it easier to maintain and extend the parameter set.

2. **Separation of Concerns**:
   - `UbxCfgParameter`: Encapsulates parameter attributes and provides value conversion/validation
   - `UbxCfgParameterLoader`: Handles loading and filtering parameters from JSON files
   - `UbxCfgHandler`: Manages parameter operations with the device and ROS integration

3. **Firmware Version Support**:
   - Parameters include firmware version information (since, until, behavior_changes)
   - The system automatically detects device firmware version
   - Parameters are filtered based on firmware compatibility
   - Behavior changes are tracked and reported to users

4. **Robust Error Handling**:
   - Comprehensive validation of parameter values
   - Clear error messages for invalid configurations
   - Graceful handling of missing or invalid parameter files

5. **Efficient Implementation**:
   - Parameters are indexed by both name and key ID for fast lookup
   - Parameter values are cached to reduce device communication
   - Firmware version comparison is optimized for performance

6. **Testability**:
   - All classes are designed for unit testing
   - Mock objects are used for testing device communication
   - Test coverage includes edge cases and error conditions

### 3. Integration with UbloxDGNSSNode

The existing node will be updated to:
- Add a device type parameter to specify ZED-F9P or ZED-F9R
- Initialize the UbxCfgHandler with the appropriate device type
- Replace the current parameter handling with the new system
- Maintain backward compatibility for existing parameter names and formats
- Add a firmware version parameter to filter parameters by firmware compatibility

## Implementation Strategy

The implementation will follow these phases:

### Phase 1: Parameter Definition File Creation
1. Define JSON schema for parameter definitions
2. Create utility scripts for schema validation
3. Extract currently supported parameters from the code
4. Create initial JSON parameter file with current parameters
5. Validate the initial parameter file against the schema

### Phase 2: Core Classes Implementation
1. Implement `UbxCfgParameter` structure
2. Implement `UbxCfgParameterLoader` class
3. Implement `UbxCfgHandler` class
4. Write unit tests for all new classes

### Phase 3: Integration
1. Update `UbloxDGNSSNode` to use the new parameter system
2. Ensure backward compatibility
3. Update build system to include JSON files
4. Add firmware version parameter to filter parameters

### Phase 4: Complete Parameter Set
1. Extract all parameters from ZED-F9P interface manual
2. Extract all parameters from ZED-F9R interface manual
3. Validate the complete parameter file

### Phase 5: Documentation and Cleanup
1. Update documentation
2. Remove deprecated code
3. Final code review and cleanup

## Technical Considerations

### JSON Parsing
- We will use the nlohmann/json library for JSON parsing
- This library is header-only and easy to integrate

### Parameter Validation
- The JSON schema will include constraints for parameter validation
- The UbxCfgParameterLoader will validate parameters during loading

### Backward Compatibility
- The new system will maintain compatibility with existing parameter names
- A migration path will be provided for users of the old system

### Error Handling
- Robust error handling will be implemented for JSON parsing errors
- Clear error messages will be provided for parameter validation failures

## Testing Strategy

1. **Unit Tests**:
   - Test parameter loading from JSON
   - Test parameter filtering by device type
   - Test value conversion between ROS and UBX types

2. **Integration Tests**:
   - Test parameter registration with ROS
   - Test parameter changes triggering UBX messages
   - Test retrieving parameter values from the device

3. **System Tests**:
   - Test with actual ZED-F9P and ZED-F9R devices
   - Verify all parameters work as expected

## Timeline

- Phase 1: 1 week
- Phase 2: 2 weeks
- Phase 3: 1 week
- Phase 4: 2 weeks
- Phase 5: 1 week

Total estimated time: 7 weeks
