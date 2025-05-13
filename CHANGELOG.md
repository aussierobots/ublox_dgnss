# Changelog

## [Unreleased]

### Fixed
- Resolved environment issues for running tests
  - Created development environment setup script (`setup_dev_env.sh`)
  - Fixed library path issues for test execution
- Fixed logical issues in UBX-CFG parameter system
  - Corrected firmware version support check to properly handle "until" version
  - Enhanced enum value validation to check against max_value
  - Updated test expectations to match implementation behavior
- Fixed code quality issues
  - Eliminated signedness comparison warnings in `ubx_cfg_handler.cpp`

### Added
- Development environment setup script (`setup_dev_env.sh`) for consistent environment setup

## [0.1.0] - 2025-05-13

### Added
- Initial release with UBX-CFG parameter system
