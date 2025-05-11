# UBX-CFG Configuration System Refactoring Tasks

## Task List

This document tracks the specific tasks needed to implement the data-driven UBX-CFG parameter handling system as described in PLANNING.md.

### Phase 1: Parameter Definition File Creation

- [ ] Define JSON schema for parameter definitions
- [ ] Create utility scripts for schema validation
- [ ] Extract currently supported parameters from code
- [ ] Create initial JSON parameter file with current parameters
- [ ] Validate the initial parameter file against the schema

### Phase 2: Core Classes Implementation

- [ ] Implement `UbxCfgParameter` structure
  - [ ] Define parameter properties
  - [ ] Implement utility methods (storage size, type conversion, etc.)
  
- [ ] Implement `UbxCfgParameterLoader` class
  - [ ] Implement JSON parsing
  - [ ] Implement parameter lookup by name and key ID
  - [ ] Implement device-specific filtering
  
- [ ] Implement `UbxCfgHandler` class
  - [ ] Implement parameter registration with ROS
  - [ ] Implement parameter value conversion
  - [ ] Implement UBX message creation and handling
  
- [ ] Write unit tests for all new classes

### Phase 3: Integration

- [ ] Update `UbloxDGNSSNode` to use the new parameter system
  - [ ] Add device type parameter
  - [ ] Initialize parameter handler
  - [ ] Set up parameter callbacks
  
- [ ] Ensure backward compatibility
  - [ ] Support existing parameter names and formats
  - [ ] Handle migration of parameter values
  
- [ ] Update build system
  - [ ] Add dependencies for JSON parsing
  - [ ] Configure installation of parameter files

### Phase 4: Complete Parameter Set

- [ ] Extract all parameters from ZED-F9P interface manual (UBX-22008968)
  - [ ] Identify all UBX-CFG parameters
  - [ ] Document parameter properties
  - [ ] Add to parameter file
  
- [ ] Extract all parameters from ZED-F9R interface manual (UBX-22010984)
  - [ ] Identify all UBX-CFG parameters
  - [ ] Document parameter properties
  - [ ] Add to parameter file
  
- [ ] Validate the complete parameter file
  - [ ] Check for duplicates
  - [ ] Verify key IDs
  - [ ] Test with both device types

### Phase 5: Documentation and Cleanup

- [ ] Update README.md with new parameter system information
- [ ] Create detailed documentation for parameter file format
- [ ] Document how to add new parameters
- [ ] Create examples for common use cases
- [ ] Remove deprecated code
- [ ] Final code review and cleanup

## Progress Tracking

| Date | Task | Status | Notes |
|------|------|--------|-------|
| 2025-05-11 | Initial planning | Completed | Created PLANNING.md and TASK.md |

## Discovered During Work

This section will be used to track additional tasks discovered during development.

## Completion Criteria

The refactoring will be considered complete when:

1. All tasks are marked as completed
2. The system successfully handles all UBX-CFG parameters for both ZED-F9P and ZED-F9R
3. Documentation is updated to explain the new system
4. All tests pass
5. Code review is completed
