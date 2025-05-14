# UBX-CFG Handler Refactoring

## Problem Statement

The current implementation of the UBX-CFG handler system has several issues that make testing difficult:

1. **Tight Coupling with USB Connection**: The `UbxCfgHandler` class is tightly coupled with the `usb::Connection` class, making it difficult to test in isolation without actual USB hardware.

2. **Segmentation Faults in Tests**: Attempts to mock the `usb::Connection` class have resulted in segmentation faults during test execution.

3. **Shared Library Issues**: Tests are failing to find required shared libraries at runtime (specifically `libublox_ubx_msgs__rosidl_typesupport_cpp.so`).

4. **Linting Issues**: There are various linting issues (cpplint, lint_cmake, uncrustify) that need to be addressed.

## Previous Attempts

1. **Direct Mocking of USB Connection**: We attempted to directly mock the `usb::Connection` class using Google Mock, but this approach led to segmentation faults, likely due to improper initialization or memory management issues.

2. **RPATH Configuration**: We updated the CMake configuration to set RPATH for shared libraries, but this did not fully resolve the runtime library loading issues.

3. **Test Simplification**: We tried to simplify the tests to focus on the core functionality, but the tight coupling with the USB connection continued to cause problems.

## Current Approach

We are now implementing a more robust solution by applying the Dependency Inversion Principle:

1. **Create a UbxTransceiver Interface**: Define an abstract interface that encapsulates the communication with the UBX device, regardless of the underlying transport mechanism (USB, serial, etc.).

2. **Implement USB-specific Transceiver**: Create a concrete implementation of the interface that uses the `usb::Connection` class.

3. **Modify UbxCfgHandler**: Update the `UbxCfgHandler` class to depend on the interface rather than directly on the `usb::Connection` class.

4. **Create Mock Implementation for Testing**: Develop a mock implementation of the interface that can be easily controlled in tests.

## Benefits of This Approach

1. **Improved Testability**: Tests can use a mock implementation of the interface, eliminating the need for actual USB hardware.

2. **Separation of Concerns**: The `UbxCfgHandler` class will focus on configuration handling, not USB communication details.

3. **Flexibility**: The system will be more flexible, allowing for different communication methods (USB, serial, network, etc.) in the future.

4. **Maintainability**: The code will be more maintainable, with clearer boundaries between components.

## Implementation Plan

1. ✅ Define the `UbxTransceiver` interface with the minimal set of methods needed by the `UbxCfgHandler`.
2. ✅ Create a `UsbUbxTransceiver` implementation that wraps the existing `usb::Connection` class.
3. ✅ Modify the `UbxCfgHandler` to use the interface instead of the concrete `usb::Connection` class.
4. ✅ Create a mock implementation of the interface for testing.
5. ✅ Update the tests to use the mock implementation.
6. ⏳ Address any remaining issues with shared libraries and linting.

## Status and Progress

The refactoring has made significant progress. The following tasks have been completed:

1. **Interface Definition**: Defined and implemented the `UbxTransceiver` interface with methods for sending/receiving UBX messages and handling ACK/NAK responses.

2. **USB Implementation**: Successfully implemented `UsbUbxTransceiver` that wraps the existing `usb::Connection` class to provide the interface functionality.

3. **UbxCfgHandler Refactoring**: Modified the `UbxCfgHandler` class to depend on the `UbxTransceiver` interface instead of directly on the `usb::Connection` class.

4. **Mock Implementation**: Created and properly implemented the `MockUbxTransceiver` class for testing purposes, ensuring it correctly implements the entire interface with matching method signatures.

5. **Test Updates**: Created a new test file `test_ubx_cfg_handler_new.cpp` that uses the mock implementation to test the `UbxCfgHandler` class without requiring actual USB hardware.

6. **Factory Implementation**: Created a `UbxTransceiverFactory` to instantiate the appropriate implementation based on context.

7. **Include Path Structure**: Fixed the include path structure for `ubx_cfg_item.hpp` to ensure it can be found by tests and implementation files.

8. **Namespace Consistency**: Updated headers with consistent namespace usage, removing unnecessary scope resolution operators.

9. **Shared Library Resolution**: Implemented comprehensive RPATH settings at the project level to address shared library loading issues.

10. **Build Stability**: Successfully achieved a clean build without any error messages, with linter checks passing.

11. **UbxCfgParameter Test Refactoring**: Updated `test_ubx_cfg_parameter.cpp` to use original enum identifiers from the UBX configuration header instead of friendly names, fixed format inconsistencies, and improved test clarity.

12. **Python Extractor Improvements**: Updated `extract_parameters.py` to ensure it preserves original enum identifiers when extracting values from the header file.

13. **UbxCfgParameterLoader Tests**: Successfully implemented and fixed tests for the `UbxCfgParameterLoader` class in a new test file `test_ubx_cfg_parameter_loader.cpp`. All tests are now passing.

14. **UbxCfgHandler Tests**: Fixed the `test_ubx_cfg_handler_new.cpp` file to properly test the UbxCfgHandler class, including:
    - Corrected UBX-MON-VER message mock implementation with proper UBX protocol framing
    - Fixed firmware version extraction and validation in test fixtures
    - Implemented proper mocking for device-specific parameter applicability tests
    - Added comprehensive tests for parameter initialization and error handling

### Remaining Tasks

1. ✅ **Parameter Test Refactoring**: Successfully updated the `test_ubx_cfg_parameter.cpp` test file and fixed all failing tests.

2. ✅ **UbxCfgParameterLoader Test Implementation**: Created and fixed the test file `test_ubx_cfg_parameter_loader.cpp` to properly test parameter loading functionality, with all tests now passing.

3. ✅ **UbxCfgHandler Testing**: Completed implementation and fixed all issues with testing the `UbxCfgHandler` class using the `MockUbxTransceiver`. All tests now pass successfully.

4. **UbxTransceiverFactory Tests**: Create and implement tests for the `UbxTransceiverFactory` class to ensure it correctly creates instances of the appropriate transceiver implementation.

5. **Test Context Implementation**: Implement or update the `Context` class referenced in the test files to properly work with the new architecture.

6. **Comprehensive Test Coverage**: Ensure all refactored components have adequate test coverage, including edge cases and error conditions.

7. **Documentation Updates**: Complete the documentation update to reflect the new architecture and interfaces, including class diagrams if applicable.

8. **Code Reviews**: Conduct final code reviews to ensure all refactored code follows project standards and best practices.

9. **Legacy Test Cleanup**: Decide whether to update or remove legacy test files that may no longer be needed with the new testing approach.

## Future Considerations

1. **Additional Transport Mechanisms**: The interface-based approach will make it easier to add support for other transport mechanisms in the future (e.g., serial, network).

2. **Enhanced Testing**: With the decoupled design, we can implement more comprehensive testing of the `UbxCfgHandler` class, including edge cases and error conditions.

3. **Performance Optimization**: The interface-based approach may introduce a small performance overhead, which could be optimized if needed.

4. **Extended Mock Capabilities**: The mock implementation could be enhanced to simulate various device behaviors for more thorough testing.
