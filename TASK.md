# UBX-CFG Configuration System Refactoring Tasks

## Task List

This document tracks the specific tasks needed to implement the data-driven UBX-CFG parameter handling system as described in PLANNING.md.

### Phase 1: Parameter Definition File Creation

- [x] Define JSON schema for parameter definitions
  - [x] Create schema file with validation rules for all parameter properties
  - [x] Include support for device-specific filtering
  - [x] Add validation for parameter types, units, and possible values
  - [x] Document schema structure and constraints
- [x] Create utility scripts for schema validation
  - [x] Implement a Python script for validating JSON files against the schema
  - [x] Add support for reporting validation errors with clear messages
  - [x] Create a helper script to check parameter consistency
- [x] Extract currently supported parameters from code
  - [x] Parse `ubx_cfg_item.hpp` to extract all parameter definitions
  - [x] Analyze `ubxKeyCfgItemMap` to ensure all parameters are captured
  - [x] Map C++ types to JSON schema types
  - [x] Extract possible values from enumeration definitions
- [x] Create initial JSON parameter file with current parameters
  - [x] Format parameters according to the schema
  - [x] Add descriptions based on code comments and variable names
  - [x] Determine applicable device types for each parameter
  - [x] Set default values based on current implementation
- [x] Validate the initial parameter file against the schema
  - [x] Run validation script to ensure compliance
  - [x] Fix any validation errors
  - [x] Verify parameter completeness against current implementation
- [x] Add firmware version support to parameter definitions
- [x] Update extraction script to include firmware version information
- [x] Validate parameter files with firmware version information

### Phase 2: Core Classes Implementation

- [ ] Implement `UbxCfgParameter` structure with firmware version support
  - [ ] Define parameter properties (name, key_id, type, scale, unit, etc.)
  - [ ] Implement utility methods for value conversion
  - [ ] Add validation methods for parameter values
  - [ ] Implement comparison operators for parameter equality
  - [ ] Add serialization/deserialization support

- [ ] Implement `UbxCfgParameterLoader` class with firmware version filtering
  - [ ] Implement JSON parsing using nlohmann/json library
  - [ ] Create efficient parameter lookup by name and key ID
  - [ ] Implement device-specific filtering
  - [ ] Add error handling for file loading and parsing errors
  - [ ] Implement parameter versioning support

- [ ] Implement `UbxCfgHandler` class
  - [ ] Implement firmware version detection
  - [ ] Implement parameter filtering based on firmware version
  - [ ] Create ROS parameter registration mechanism
  - [ ] Implement parameter value conversion between ROS and UBX types
  - [ ] Create UBX message generation for parameter get/set operations
  - [ ] Implement parameter change callback handling
  - [ ] Add support for parameter validation before setting
  - [ ] Implement parameter caching to reduce device communication

- [ ] Write unit tests for all new classes
  - [ ] Test parameter loading from JSON files
  - [ ] Test parameter filtering by device type and firmware version
  - [ ] Test value conversion between different types
  - [ ] Test error handling and edge cases
  - [ ] Test parameter validation logic

### Phase 3: Integration

- [ ] Update `UbloxDGNSSNode` to use the new parameter system
  - [ ] Add device type parameter with validation
  - [ ] Initialize parameter handler with appropriate device type
  - [ ] Set up parameter callbacks for handling changes
  - [ ] Update parameter initialization sequence
  - [ ] Modify parameter get/set operations to use the new system

- [ ] Ensure backward compatibility
  - [ ] Create mapping between old and new parameter names if needed
  - [ ] Support existing parameter names and formats
  - [ ] Handle migration of parameter values from old to new system
  - [ ] Add deprecation warnings for old parameter usage
  - [ ] Implement fallback mechanisms for backward compatibility

- [ ] Update build system
  - [ ] Add dependencies for JSON parsing (nlohmann/json)
  - [ ] Configure installation of parameter files
  - [ ] Update CMakeLists.txt to include new source files
  - [ ] Configure package.xml with new dependencies
  - [ ] Set up proper installation paths for JSON files

### Phase 4: Complete Parameter Set

- [ ] Extract all parameters from ZED-F9P interface manual (UBX-22008968)
  - [ ] Identify all UBX-CFG parameters in the manual
  - [ ] Document parameter properties (key ID, type, scale, unit)
  - [ ] Add descriptions from the manual
  - [ ] Document possible values and their meanings
  - [ ] Add to parameter file with ZED-F9P device type

- [ ] Extract all parameters from ZED-F9R interface manual (UBX-22010984)
  - [ ] Identify all UBX-CFG parameters in the manual
  - [ ] Document parameter properties (key ID, type, scale, unit)
  - [ ] Add descriptions from the manual
  - [ ] Document possible values and their meanings
  - [ ] Add to parameter file with ZED-F9R device type
  - [ ] Identify parameters that differ from ZED-F9P

- [ ] Validate the complete parameter file
  - [ ] Check for duplicates and inconsistencies
  - [ ] Verify key IDs against manual documentation
  - [ ] Test with both device types to ensure compatibility
  - [ ] Verify all parameters can be loaded and used
  - [ ] Check for any missing parameters

### Phase 5: Documentation and Cleanup

- [ ] Update README.md with new parameter system information
  - [ ] Explain the data-driven approach
  - [ ] Document how to configure the device type
  - [ ] Provide examples of parameter usage

- [ ] Create detailed documentation for parameter file format
  - [ ] Document JSON schema structure
  - [ ] Explain parameter properties and their meanings
  - [ ] Provide examples of parameter definitions

- [ ] Document how to add new parameters
  - [ ] Create step-by-step guide for adding parameters
  - [ ] Explain validation process
  - [ ] Document common pitfalls and solutions

- [ ] Create examples for common use cases
  - [ ] Basic parameter configuration
  - [ ] Device-specific parameter sets
  - [ ] Parameter validation and error handling

- [ ] Remove deprecated code
  - [ ] Remove hardcoded parameter definitions
  - [ ] Clean up old parameter handling code
  - [ ] Update comments and documentation

- [ ] Final code review and cleanup
  - [ ] Check for code quality and style
  - [ ] Verify error handling
  - [ ] Ensure proper memory management
  - [ ] Check for potential performance issues

## Progress Tracking

| Date | Task | Status | Notes |
|------|------|--------|-------|
| 2025-05-11 | Initial planning | Completed | Created PLANNING.md and TASK.md |
| 2025-05-13 | Code analysis | Completed | Analyzed current implementation and updated planning documents |
| 2025-05-13 | JSON schema creation | Completed | Created schema for parameter definitions with firmware version support |
| 2025-05-13 | Parameter extraction | Completed | Developed script to extract parameters from existing code |
| 2025-05-13 | Firmware version support | Completed | Added firmware version information to parameter definitions |

## Discovered During Work

### Code Analysis Findings (2025-05-13)

- The current parameter handling is tightly integrated with the ROS parameter system
- Parameter values are converted between ROS parameter types and UBX types in the `set_or_declare_ubx_cfg_param` method
- The node uses `add_on_set_parameters_callback` to handle parameter changes
- There is no explicit device type differentiation in the current implementation
- The `ubxKeyCfgItemMap` is a global map that contains all parameter definitions

## Technical Considerations

- **JSON Parsing Library**: We will use nlohmann/json for JSON parsing as it's header-only and widely used
- **Parameter Validation**: We need robust validation to ensure parameters are correctly defined and used
- **Error Handling**: Clear error messages are essential for debugging parameter issues
- **Performance**: The parameter loader should be efficient, especially for large parameter sets
- **Testing**: Comprehensive unit tests are needed to ensure reliability

## Completion Criteria

The refactoring will be considered complete when:

1. All tasks are marked as completed
2. The system successfully handles all UBX-CFG parameters for both ZED-F9P and ZED-F9R
3. Documentation is updated to explain the new system
4. All tests pass
5. Code review is completed
6. The system demonstrates improved maintainability and extensibility
