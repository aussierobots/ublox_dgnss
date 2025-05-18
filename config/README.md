# UBX-CFG Parameter System

This directory contains the configuration files and utilities for the UBX-CFG parameter system used in the ublox_dgnss driver.

## Overview

The UBX-CFG parameter system uses a data-driven approach to manage configuration parameters for u-blox GNSS receivers. Instead of hardcoding parameters in the source code, we define them in TOML files that can be loaded at runtime. This approach offers several advantages:

1. **Maintainability**: Parameters can be updated without changing the code
2. **Extensibility**: New parameters can be added easily
3. **Device-specific filtering**: Parameters can be filtered by device type
4. **Firmware compatibility**: Parameters can be filtered by firmware version

## Directory Structure

- `schema/`: Contains schema and validation utilities
  - `validate_toml_parameters.py`: Script to validate TOML parameter files
  - `extract_parameters_toml.py`: Script to extract parameters from C++ code to TOML format
  - `validate_toml.sh`: Wrapper script for validating TOML files
  - `extract_toml.sh`: Wrapper script for extracting parameters to TOML
- `ubx_cfg_parameters_sample.toml`: Sample parameter file with a subset of parameters
- `ubx_cfg_parameters_full.toml`: Complete parameter file with all supported parameters

## Parameter File Format

The parameter files now use the TOML format, which offers better readability and comment support. Here's the structure:

```toml
# UBX-CFG Parameters Configuration File
# TOML version of the configuration

# Basic version information
version = "1.0.0"
device_types = ["ZED-F9P", "ZED-F9R"]

# Firmware version information
[firmware_versions]

# ZED-F9P firmware versions
[[firmware_versions.ZED-F9P]]
version = "HPG 1.13"
description = "Early production release"
release_date = "2020-06-15"

[[firmware_versions.ZED-F9P]]
version = "HPG 1.30"
description = "Production release with improved RTK performance"
release_date = "2022-01-15"

# ZED-F9R firmware versions
[[firmware_versions.ZED-F9R]]
version = "HPS 1.13"
description = "Early production release"
release_date = "2020-08-10"

[[firmware_versions.ZED-F9R]]
version = "HPS 1.30"
description = "Latest production release with enhanced dead reckoning"
release_date = "2024-01-14"

# Parameters configuration

# Parameter: CFG_INFMSG_UBX_USB
[[parameters]]
name = "CFG_INFMSG_UBX_USB"
key_id = "0x20920004"
type = "X1"
scale = 1.0
unit = "NA"
applicable_devices = ["ZED-F9P", "ZED-F9R"]
description = "Information message configuration for UBX protocol on USB"
group = "INFMSG"
default_value = "0x00"

[parameters.possible_values]
INFMSG_ERROR = "0x01"
INFMSG_WARNING = "0x02"
INFMSG_NOTICE = "0x04"
INFMSG_TEST = "0x08"
INFMSG_DEBUG = "0x10"

[parameters.firmware_support.ZED-F9P]
since = "HPG 1.13"

[parameters.firmware_support.ZED-F9P.behavior_changes]
version = "HPG 1.30"
description = "Default value changed to 0x01"

[parameters.firmware_support.ZED-F9R]
since = "HPS 1.13"
until = "HPS 1.30"
```

## Firmware Version Support

The parameter system includes support for firmware versioning to handle compatibility between parameters and device firmware. Each parameter includes a `firmware_support` field that specifies:

1. **since**: The firmware version when the parameter was introduced
2. **until** (optional): The firmware version when the parameter was deprecated
3. **behavior_changes** (optional): A list of firmware versions where the parameter's behavior changed

This information allows the driver to:
- Filter out parameters not supported by the current firmware
- Provide warnings when using parameters with changed behavior
- Handle deprecated parameters gracefully

## Usage

### Validation

Validate TOML parameter files using our Pydantic-based validation tool:

```bash
./config/schema/validate_toml.sh config/ubx_cfg_parameters_full.toml
```

This validates that your TOML file conforms to the expected schema and provides detailed error messages when it doesn't.

### Parameter Extraction

To extract parameters directly from C++ code to TOML format:

```bash
./config/schema/extract_toml.sh ublox_dgnss_node/include/ublox_dgnss_node/ubx/ubx_cfg_item.hpp config/ubx_cfg_parameters_full.toml
```

This uses our TOML extraction tool to analyze the C++ header files and output a complete parameter file in TOML format.



## Tool Dependencies

Our TOML tools require the following Python packages:

- `pydantic`: For schema validation
- `tomli`: For TOML parsing (Python < 3.11)
- `tomli_w`: For TOML writing

These dependencies are automatically managed through our virtual environment setup. The wrapper scripts (`validate_toml.sh` and `extract_toml.sh`) handle this for you.

## Adding New Parameters

To add a new parameter to the TOML configuration file:

1. Open the TOML file (`ubx_cfg_parameters_full.toml`) in your editor of choice

2. Add a new parameter entry following this template:

```toml
# Parameter: YOUR_PARAMETER_NAME
[[parameters]]
name = "CFG_YOUR_PARAMETER_NAME"
key_id = "0x20XXXXXX"  # Replace with actual key ID in hex format
type = "X1"            # Use appropriate UBX type (U1, X2, etc.)
scale = 1.0
unit = "NA"            # Or appropriate unit
applicable_devices = ["ZED-F9P", "ZED-F9R"]  # Adjust as needed
description = "Description of your parameter"
group = "GROUP_NAME"    # e.g., INFMSG, NAVSPG, etc.
default_value = "0x00"  # Default value in appropriate format

# Optional: Add possible values if parameter is an enumeration
[parameters.possible_values]
VALUE_NAME_1 = "0x01"
VALUE_NAME_2 = "0x02"

# Add firmware support information for each applicable device
[parameters.firmware_support.ZED-F9P]
since = "HPG 1.13"      # Firmware version when parameter was introduced
# Optional: Add until field if parameter is deprecated in a specific version
# until = "HPG 1.32"

# Optional: Add behavior changes if relevant
[[parameters.firmware_support.ZED-F9P.behavior_changes]]
version = "HPG 1.30"
description = "Behavior change description"

# Add firmware support for other devices
[parameters.firmware_support.ZED-F9R]
since = "HPS 1.13"
```

3. Validate your updated file:

```bash
./config/schema/validate_toml.sh config/ubx_cfg_parameters_full.toml
```

4. Test the parameter with the UBX-CFG system to ensure it's properly recognized and processed

5. Update any relevant code documentation to reflect the new parameter
4. Restart the driver to load the new parameter

## Future Enhancements

- Support for more device types
- More detailed firmware version compatibility information
- Automatic parameter documentation generation
