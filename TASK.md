# UBX-CFG Configuration System Refactoring Tasks

## Task List

This document tracks the specific tasks needed to implement the data-driven UBX-CFG parameter handling system as described in PLANNING.md.

### Phase 1: Parameter Definition File Creation

- [x] Define parameter structure for configuration files
  - [x] Create validation rules for all parameter properties
  - [x] Include support for device-specific filtering
  - [x] Add validation for parameter types, units, and possible values
  - [x] Document structure and constraints
- [x] Create utility scripts for validation
  - [x] Implement a Python script for validating parameter files
  - [x] Add support for reporting validation errors with clear messages
  - [x] Create a helper script to check parameter consistency
- [x] Extract currently supported parameters from code
  - [x] Parse `ubx_cfg_item.hpp` to extract all parameter definitions
  - [x] Analyze `ubxKeyCfgItemMap` to ensure all parameters are captured
  - [x] Map C++ types to appropriate TOML types
  - [x] Extract possible values from enumeration definitions
- [x] Create initial parameter file with current parameters
  - [x] Format parameters according to the TOML structure
  - [x] Add descriptions based on code comments and variable names
  - [x] Determine applicable device types for each parameter
  - [x] Set default values based on current implementation
- [x] Validate the initial parameter file
  - [x] Run validation script to ensure compliance
  - [x] Fix any validation errors
  - [x] Verify parameter completeness against current implementation
- [x] Add firmware version support to parameter definitions
- [x] Update extraction script to include firmware version information
- [x] Validate parameter files with firmware version information

### Phase 2: Core Classes Implementation

- [x] Implement `UbxCfgParameter` structure with firmware version support
  - [x] Define parameter properties (name, key_id, type, scale, unit, etc.)
  - [x] Implement utility methods for value conversion
  - [x] Add validation methods for parameter values
  - [x] Implement comparison operators for parameter equality
  - [x] Add firmware version comparison logic

- [x] Implement `UbxCfgParameterLoader` class with firmware version filtering
  - [x] Implement TOML parsing using toml11 library
  - [x] Create efficient parameter lookup by name and key ID
  - [x] Implement device-specific filtering
  - [x] Add error handling for file loading and parsing errors
  - [x] Implement parameter versioning support
  - [x] Add firmware version filtering capabilities

- [x] Implement `UbxCfgHandler` class
  - [x] Implement firmware version detection
  - [x] Create parameter registration with ROS
  - [x] Implement parameter change handling
  - [x] Add parameter value validation
  - [x] Implement parameter behavior change tracking
  - [x] Create UBX message generation for parameter get/set operations
  - [x] Implement parameter change callback handling
  - [x] Add support for parameter validation before setting
  - [x] Implement parameter caching to reduce device communication

- [x] Write unit tests for all new classes
  - [x] Test parameter loading from TOML files
  - [x] Test parameter filtering by device type and firmware version
  - [x] Test value conversion between different types
  - [x] Test error handling and edge cases
  - [x] Test parameter validation logic

- [x] Fix test environment issues
  - [x] Resolve shared library loading issues in test environment
  - [x] Fix logical test failures in UbxCfgParameter tests
  - [x] Fix logical test failures in UbxCfgParameterLoader tests
  - [x] Create development environment setup script
  - [x] Fix signedness comparison warnings
  - [x] Fix UbxCfgHandler test issues
    - [x] Implement proper UBX-MON-VER mock response with correct framing
    - [x] Fix firmware version extraction and validation
    - [x] Fix parameter initialization and device-specific applicability tests

- [x] Improve test coverage and documentation
  - [x] Refactor test_ubx_cfg_parameter.cpp to use original enum identifiers
  - [x] Fix format inconsistencies in test value representation (using hex format)
  - [x] Implement comprehensive tests for UbxCfgParameterLoader in test_ubx_cfg_parameter_loader.cpp
  - [x] Add detailed diagnostics to test cases to aid in debugging
  - [x] Document parameter handling behavior with nonexistent firmware versions

### Phase 2.5: TOML Configuration Implementation

- [x] Research and select a TOML parsing library
  - [x] Evaluate toml11 and cpptoml libraries
  - [x] Test basic parsing functionality
  - [x] Verify compatibility with ROS 2 build system

- [x] Update build system for TOML support
  - [x] Add dependency on toml11 library in CMakeLists.txt
  - [x] Update package.xml if needed
  - [x] Configure FetchContent to download the library if not found

- [x] Create TOML parameter files
  - [x] Create sample parameter file
  - [x] Create full parameter file
  - [x] Ensure proper TOML structure

- [x] Implement UbxCfgParameterLoader for TOML
  - [x] Use toml11 library for parsing
  - [x] Design method signatures for TOML types
  - [x] Implement parsing logic for TOML structure
  - [x] Optimize for TOML's hierarchical organization

- [x] Create tests for TOML integration
  - [x] Create test data files in TOML format
  - [x] Implement test_ubx_cfg_parameter_loader.cpp for TOML
  - [x] Implement test_ubx_cfg_handler_new.cpp for TOML
  - [x] Test parsing of TOML values
  - [x] Add tests for TOML-specific features

- [x] Create documentation for TOML format
  - [x] Update README with TOML examples
  - [x] Document parameter file structure
  - [x] Provide guidance for adding new parameters

- [x] Create validation tools for TOML format
  - [x] Implement Pydantic-based validation script
  - [x] Create virtual environment for dependencies
  - [x] Create wrapper scripts for easy usage
  - [x] Test validation against parameter files

- [x] Create parameter extraction tools for TOML format
  - [x] Create parameter extraction script that outputs to TOML
  - [x] Install required dependencies in virtual environment
  - [x] Add wrapper script for easy usage
  - [x] Test extraction with existing header files

### Phase 3: Integration

- [ ] Update `UbloxDGNSSNode` to use the new parameter system
  - [ ] Add device type parameter with validation
  - [ ] Initialize parameter handler with appropriate device type
  - [ ] Set up parameter callbacks for handling changes
  - [ ] Update parameter initialization sequence
  - [ ] Modify parameter get/set operations to use the new system

- [ ] Ensure robust parameter handling
  - [ ] Validate parameter names and formats
  - [ ] Implement comprehensive error checking
  - [ ] Add detailed logging for parameter operations
  - [ ] Create parameter documentation in code

- [ ] Update build system
  - [ ] Add dependencies for TOML parsing (toml11)
  - [ ] Configure installation of parameter files
  - [ ] Update CMakeLists.txt to include new source files
  - [ ] Configure package.xml with new dependencies
  - [ ] Set up proper installation paths for TOML files

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
  - [ ] Document TOML structure
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
| 2025-05-14 | UbxCfgParameter test refactoring | Completed | Updated tests to use original enum identifiers and consistent formatting |
| 2025-05-14 | UbxCfgParameterLoader tests | Completed | Fixed JSON structure and implemented comprehensive tests for parameter loading functionality |
| 2025-05-17 | Decision to use TOML instead of JSON | In Progress | Determined TOML offers better readability and maintainability for configuration files |

## Discovered During Work

### Code Analysis Findings (2025-05-13)

- The current parameter handling is tightly integrated with the ROS parameter system
- Parameter values are converted between ROS parameter types and UBX types in the `set_or_declare_ubx_cfg_param` method
- The node uses `add_on_set_parameters_callback` to handle parameter changes
- There is no explicit device type differentiation in the current implementation
- The `ubxKeyCfgItemMap` is a global map that contains all parameter definitions

### UbxCfgParameterLoader Behavior (2025-05-14)

- The `UbxCfgParameterLoader::get_parameters_for_device_and_firmware` method returns parameters for nonexistent firmware versions as long as the device type is valid
- This behavior appears to be intentional to support unknown or future firmware versions
- When a nonexistent firmware version is specified, all parameters applicable to the device type are returned
- This can be useful for handling cases where a device reports an unknown firmware version
- The test suite now includes validation of this behavior to ensure consistency

## Technical Considerations

- **TOML Parsing Library**: We will use toml11 for TOML parsing as it's header-only and provides a similar API to nlohmann/json
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
