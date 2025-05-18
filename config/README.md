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
  - `ubx_cfg_parameters_schema.json`: Schema definition for parameter files (legacy JSON format)
  - `validate_parameters.py`: Script to validate parameter files against the schema
  - `extract_parameters.py`: Script to extract parameters from the C++ code
- `ubx_cfg_parameters_sample.toml`: Sample parameter file with a subset of parameters
- `ubx_cfg_parameters_full.toml`: Complete parameter file with all supported parameters
- Legacy JSON files are also maintained for backward compatibility:
  - `ubx_cfg_parameters_sample.json`: Legacy JSON sample parameter file
  - `ubx_cfg_parameters_full.json`: Legacy JSON complete parameter file

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

We're transitioning validation scripts to support TOML. For now, you can still validate the legacy JSON files using:

```bash
python config/schema/validate_parameters.py config/ubx_cfg_parameters_full.json
```

Note: We'll be updating the validation utilities to work directly with TOML files in future releases.

### Parameter Extraction

To extract parameters from C++ code (output still in JSON format, but can be converted to TOML):

```bash
python config/schema/extract_parameters.py ublox_dgnss_node/include/ublox_dgnss_node/ubx/ubx_cfg_item.hpp config/ubx_cfg_parameters_full.json
```

You can convert the resulting JSON to TOML using the provided conversion script:

```bash
python scripts/simple_json_to_toml.py config/ubx_cfg_parameters_full.json config/ubx_cfg_parameters_full.toml
```

## Adding New Parameters

To add a new parameter:

1. Add the parameter to the JSON file following the schema
2. Include firmware version information for each supported device
3. Validate the updated file using the validation script
4. Restart the driver to load the new parameter

## Future Enhancements

- Support for more device types
- More detailed firmware version compatibility information
- Automatic parameter documentation generation
