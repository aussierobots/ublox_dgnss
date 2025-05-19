# TOML Parameter Implementation Strategy

## Current Status (Updated May 18, 2025)

We're implementing TOML configuration functionality in the `UbloxDGNSSNode` class as part of the transition from JSON to TOML configuration for the UBX-CFG system. Here's our current progress:

### Completed Tasks

1. **Device Type Parameter Integration**
   - Added device type parameter constants to the `UbloxDGNSSNode` class
   - Implemented device type storage as a class member variable
   - Created validation functions for device type parameters
   - Modified the constructor to declare, validate, and use the device type parameter

2. **Build System Fixes**
   - Fixed compilation issues related to parameter handling
   - Successfully built the project with the new device type parameter support
   - Temporarily disabled the `UbxCfgHandler` initialization to avoid linking errors

3. **UbxCfgHandler Interface Updates**
   - Added the `update_device_type` method to the `UbxCfgHandler` class
   - Added the `has_device_type` method to `UbxCfgParameterLoader` class
   - Prepared the code structure for future integration

### Context

### File Locations

- **Main Node Implementation**: `/home/geoff/ros2_ws/src/ublox_dgnss_fork/ublox_dgnss_node/src/ublox_dgnss_node.cpp`
- **TOML Configuration Files**: `/home/geoff/ros2_ws/src/ublox_dgnss_fork/ublox_dgnss_node/config/ubx_cfg_parameters_full.toml`
- **UbxCfgHandler**: `/home/geoff/ros2_ws/src/ublox_dgnss_fork/ublox_dgnss_node/include/ublox_dgnss_node/ubx/cfg/ubx_cfg_handler.hpp`

### Class Structure

The `UbloxDGNSSNode` class serves as the main ROS 2 node for the u-blox DGNSS device communication. It initializes the device, handles parameters, and processes messages. The class already declares some parameters like `serial_str` and `frame_id`, but we need to add device type parameter handling.

### Integration with UbxCfgHandler

The device type parameter needs to be passed to the UbxCfgHandler, which handles loading and processing the TOML configuration files. The UbxCfgHandler is initialized in the UbloxDGNSSNode constructor and is used to filter configuration parameters based on device type.

## Issues Identified

1. **Code Structure Issues**:
   - Many functions in `ublox_dgnss_node.cpp` are defined without proper class qualification
   - Functions that should be class member functions are defined as standalone functions
   - Inconsistent use of logger calls (`get_logger()` vs. `this->get_logger()`)

2. **Parameter Handling**:
   - Need to implement proper parameter validation for device types
   - Need to ensure parameter constants are defined correctly

3. **Compilation Errors**:
   - Scope issues with class member functions
   - Issues with static constexpr member variables
   - Template argument errors in logger macro calls

## Solutions and Approaches

### 1. Parameter Constants Definition

The following approach works correctly for defining parameter constants:

```cpp
// Inside the class definition
private:
  // Parameter name constants
  static constexpr char DEVICE_TYPE_PARAM_NAME[] = "device_type";
  static constexpr char DEV_STRING_PARAM_NAME[] = "serial_str";
  static constexpr char FRAME_ID_PARAM_NAME[] = "frame_id";
```

Outside the class, after the class definition but before the namespace closing:

```cpp
// Define static constexpr members
constexpr char ublox_dgnss::UbloxDGNSSNode::DEVICE_TYPE_PARAM_NAME[];
constexpr char ublox_dgnss::UbloxDGNSSNode::DEV_STRING_PARAM_NAME[];
constexpr char ublox_dgnss::UbloxDGNSSNode::FRAME_ID_PARAM_NAME[];
```

### 2. Device Type Parameter Handling

The following implementation for `check_for_device_type_param` works correctly:

```cpp
void check_for_device_type_param(rclcpp::SyncParametersClient::SharedPtr param_client)
{
  // Default to ZED-F9P
  device_type_ = "ZED-F9P";
  
  // Check if the parameter exists
  if (!param_client->has_parameter(DEVICE_TYPE_PARAM_NAME)) {
    // Define parameter descriptor
    rcl_interfaces::msg::ParameterDescriptor device_type_desc;
    device_type_desc.name = DEVICE_TYPE_PARAM_NAME;
    device_type_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    device_type_desc.description = "u-blox device type - supported values: ZED-F9P, ZED-F9R";
    device_type_desc.additional_constraints = "Must be one of the supported device types";
    
    // Declare the parameter with default value
    this->declare_parameter(DEVICE_TYPE_PARAM_NAME, device_type_, device_type_desc);
    RCLCPP_INFO(this->get_logger(), "Declared device type parameter with default value: %s", device_type_.c_str());
  }
  
  // Get the parameter value
  device_type_ = param_client->get_parameter<std::string>(DEVICE_TYPE_PARAM_NAME);
  
  // Validate the device type
  if (device_type_ != "ZED-F9P" && device_type_ != "ZED-F9R") {
    RCLCPP_WARN(this->get_logger(), "Unsupported device type: %s. Setting to default: ZED-F9P", device_type_.c_str());
    device_type_ = "ZED-F9P";
    this->set_parameter(rclcpp::Parameter(DEVICE_TYPE_PARAM_NAME, device_type_));
  }
  
  RCLCPP_INFO(this->get_logger(), "Using device type: %s", device_type_.c_str());
}
```

### 3. Recommendations for Code Structure

1. **Logger Calls**: 
   - Always use `this->get_logger()` instead of `get_logger()`
   
2. **Function Definitions**:
   - Keep function definitions within the class scope if possible
   - If defined outside, ensure proper qualification with class name
   
3. **Parameter Management**:
   - Use descriptors for all parameters to provide documentation
   - Implement validation for all parameters with constraints
   - Provide safe default values for parameters

## Recommended Implementation Steps

1. **Minimal Changes Approach**:
   - Add the parameter constants to the class definition
   - Implement `check_for_device_type_param` function
   - Add static constexpr definitions outside the class
   - Call `check_for_device_type_param` from the constructor

2. **Integration with UbxCfgHandler**:
   - Make sure the device type parameter is respected in UbxCfgHandler
   - Ensure UbxCfgHandler loads the correct TOML file
   - Pass the device type to UbxCfgHandler for parameter filtering

3. **Testing**:
   - Test with both supported device types
   - Test with invalid device types to verify validation
   - Verify parameter is properly reflected in logs

## Implementation Steps

1. **Add Parameter Member Variables**:
   - Add device_type_ string member variable to UbloxDGNSSNode class
   - Add static parameter name constant DEVICE_TYPE_PARAM_NAME

2. **Implement check_for_device_type_param Function**:
   - Add function to declare, validate, and retrieve the device type parameter
   - Ensure proper validation against supported device types (ZED-F9P, ZED-F9R)

3. **Integrate with Initialization**:
   - Call the parameter checking function from the constructor
   - Pass the device type to UbxCfgHandler when initializing

4. **Update UbxCfgHandler Integration**:
   - Modify UbxCfgHandler initialization to use the device type for parameter filtering
   - Ensure device-specific parameters are properly applied

## Testing Approach

1. **Manual Testing**:
   - Test with valid device types (ZED-F9P, ZED-F9R)
   - Test with invalid device type to verify validation
   - Verify parameter appears in ROS parameter list

2. **Testing with Hardware**:
   - Test with actual hardware to ensure correct device configuration
   - Verify device type influences behavior as expected

3. **Unit Test Creation**:
   - Create a unit test to validate parameter handling
   - Validate parameter constraints work correctly

## Build and Run Instructions

1. After making changes, build using:
   ```bash
   cd /home/geoff/ros2_ws
   colcon build --packages-select ublox_dgnss_node
   ```

2. Test the node using:
   ```bash
   source install/setup.bash
   ros2 run ublox_dgnss_node ublox_dgnss --ros-args -p device_type:=ZED-F9P
   ```

## Progress Update (May 19, 2025)

### Recently Completed

1. **UbxTransceiverFactory Integration**
   - Fixed linking errors with `UbxTransceiverFactory::create_usb_transceiver` method
   - Created a simplified `SimpleUbxTransceiver` implementation in the factory source file
   - Successfully integrated with `UbxCfgHandler` initialization in the UbloxDGNSSNode class
   - The build now completes successfully with all components properly linked

### Outstanding Issues

1. **TOML File Format Conversion**
   - Pointing to TOML file path (`ubx_cfg_parameters_full.toml`), but need to verify all configuration files are properly converted
   - Need to validate TOML parsing functionality in the parameter loader

2. **Test Environment Issues**
   - Tests still failing with runtime errors - can't find required shared libraries
   - Need to ensure correct path setup for `libublox_ubx_msgs__rosidl_generator_c.so`

3. **UbxTransceiver Implementation**
   - Current implementation is a simplified mock implementation
   - Need to implement full functionality for production use

## Next Steps

1. **Verify UbxCfgHandler Integration**
   - Test the integration between UbloxDGNSSNode and UbxCfgHandler
   - Verify that device type parameters are correctly passed and applied
   - Test the parameter callback system for handling device type changes

2. **Implement Full UbxTransceiver Functionality**
   - Revisit the SimpleUbxTransceiver implementation to add real functionality
   - Transition from mock implementation to a fully functional implementation
   - Consider refactoring UsbUbxTransceiver to properly implement the UbxTransceiver interface

3. **Finalize TOML Conversion**
   - Verify all JSON configuration files have been converted to TOML format
   - Test TOML parsing and validation functionality
   - Update any remaining code that expects JSON files

4. **Test Environment Fixes**
   - Set up proper paths for shared libraries using `setup_dev_env.sh`
   - Ensure tests can find `libublox_ubx_msgs__rosidl_generator_c.so`
   - Re-enable the disabled tests once issues are fixed

5. **Comprehensive Testing**
   - Test with different device types (ZED-F9P, ZED-F9R)
   - Verify parameters are correctly filtered by device type
   - Ensure proper parameter validation

## Reference Implementation

A test implementation for parameter handling functionality is available that demonstrates the approach. Key features:

```cpp
// Inside class definition
private:
  // Parameter name constants
  static constexpr char DEVICE_TYPE_PARAM_NAME[] = "device_type";
  
  // Device type storage
  std::string device_type_;

  void check_for_device_type_param(rclcpp::SyncParametersClient::SharedPtr param_client) {
    // Implementation details...
  }
```

// Outside class definition (after class but before namespace end)
```cpp
// Define static constexpr members
constexpr char YourClassName::DEVICE_TYPE_PARAM_NAME[];
```
