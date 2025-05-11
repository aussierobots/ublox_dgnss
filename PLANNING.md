# UBX-CFG Configuration System Refactoring

## Project Overview

This document outlines the plan for refactoring the UBX-CFG configuration system in the ublox_dgnss driver to support a more maintainable and extensible approach to handling configuration parameters for both ZED-F9P and ZED-F9R devices.

## Reference Documents

This implementation is based on the following u-blox documents:

- ZED-F9P Interface Description (UBX-22008968)
- ZED-F9R Interface Description (UBX-22010984)

These documents can be obtained from the u-blox website:
https://www.u-blox.com/en/product-resources

## Current Implementation

The current implementation defines UBX-CFG parameters directly in C++ code:

- Each parameter is defined as a `ubx_cfg_item_t` structure in `ubx_cfg_item.hpp`
- All parameters are registered in the `ubxKeyCfgItemMap`
- The node implementation handles these parameters by declaring them as ROS parameters, retrieving values from the device, and setting values on the device when changed

This approach has several limitations:
- Adding new parameters requires modifying and recompiling code
- Supporting different device types (ZED-F9P vs ZED-F9R) is cumbersome
- Maintaining the large number of parameters available in the interface manuals is labor-intensive

## Proposed Solution

We will implement a data-driven approach using configuration files:

1. Define parameters in a structured JSON format
2. Create a parameter loader to parse the JSON file
3. Implement a parameter handler to manage the dynamic parameters
4. Update the node implementation to use the new system

### Architecture

#### 1. Parameter Definition File (JSON)

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
      }
    }
    // ... more parameters
  ]
}
```

#### 2. C++ Implementation

New classes:
- `UbxCfgParameter`: Structure to hold parameter definitions
- `UbxCfgParameterLoader`: Class to load and parse parameter files
- `UbxCfgHandler`: Class to manage dynamic parameters

#### 3. Integration

The existing UbloxDGNSSNode will be updated to:
- Use the new parameter handler
- Support device-specific filtering
- Handle dynamic parameter registration and changes

## Benefits

1. **Separation of Configuration from Code**: Parameter definitions are separate from the code that uses them
2. **Easier Maintenance**: Adding new parameters only requires updating the JSON file
3. **Device-Specific Filtering**: Parameters can be filtered based on the device type
4. **Better Documentation**: Parameter descriptions and possible values can be included
5. **Versioning**: Different versions of the parameters can be maintained
6. **Automation Potential**: Scripts could be developed to extract parameters from the interface manuals

## Implementation Strategy

The implementation will follow these phases:

1. **Parameter Definition File Creation**: Create JSON schema and initial file
2. **Core Classes Implementation**: Implement parameter handling classes
3. **Integration**: Update the node to use the new system
4. **Complete Parameter Set**: Extract all parameters from the interface manuals
5. **Documentation and Cleanup**: Update documentation and remove deprecated code

## Testing Strategy

1. **Unit Tests**: Test parameter loading, filtering, and conversion
2. **Integration Tests**: Test parameter registration and device interaction
3. **System Tests**: Test with actual devices

## Coding Standards

- Follow ROS 2 coding standards
- Use C++17 features where appropriate
- Maintain backward compatibility with existing code
- Provide comprehensive documentation

## Timeline

- Phase 1: 1 week
- Phase 2: 2 weeks
- Phase 3: 1 week
- Phase 4: 2 weeks
- Phase 5: 1 week

Total estimated time: 7 weeks
