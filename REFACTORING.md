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

## Progress Update (May 13, 2025)

### Completed Tasks

1. **Interface Definition**: Created the `UbxTransceiver` interface with methods for sending/receiving UBX messages and handling ACK/NAK responses.

2. **USB Implementation**: Implemented `UsbUbxTransceiver` that wraps the existing `usb::Connection` class to provide the interface functionality.

3. **UbxCfgHandler Refactoring**: Modified the `UbxCfgHandler` class to depend on the `UbxTransceiver` interface instead of directly on the `usb::Connection` class.

4. **Mock Implementation**: Created a `MockUbxTransceiver` class for testing purposes, allowing for controlled simulation of UBX device communication.

5. **Test Updates**: Created a new test file `test_ubx_cfg_handler_new.cpp` that uses the mock implementation to test the `UbxCfgHandler` class without requiring actual USB hardware.

6. **Factory Implementation**: Created a `UbxTransceiverFactory` to instantiate the appropriate implementation based on context.

### Remaining Tasks

1. **Test Execution**: Run the new tests to verify that the refactored code works as expected.

2. **Shared Library Issues**: Continue addressing issues with shared library loading at runtime.

3. **Linting**: Address remaining linting issues across the codebase.

4. **Documentation**: Update documentation to reflect the new architecture and interfaces.

5. **Legacy Test Cleanup**: Once the new tests are confirmed to be working, consider removing or disabling the old test file.

## Future Considerations

1. **Additional Transport Mechanisms**: The interface-based approach will make it easier to add support for other transport mechanisms in the future (e.g., serial, network).

2. **Enhanced Testing**: With the decoupled design, we can implement more comprehensive testing of the `UbxCfgHandler` class, including edge cases and error conditions.

3. **Performance Optimization**: The interface-based approach may introduce a small performance overhead, which could be optimized if needed.

4. **Extended Mock Capabilities**: The mock implementation could be enhanced to simulate various device behaviors for more thorough testing.
