#!/usr/bin/env python3
"""
UBX-CFG Parameter Extractor (TOML Version)

This script extracts UBX-CFG parameters from the ubx_cfg_item.hpp file and
converts them to the TOML format defined by our schema.

Usage:
  python extract_parameters_toml.py <ubx_cfg_item.hpp> <output.toml>
"""

import re
import sys
import os
from typing import Dict, List, Any, Optional
from datetime import datetime

try:
    import tomli_w  # Python 3.7+ TOML writer
except ImportError:
    # Attempt to install if missing
    print("tomli_w package is required. Please install it with: pip install tomli_w")
    sys.exit(1)

def extract_parameters(hpp_file: str) -> List[Dict[str, Any]]:
    """Extract parameters from the ubx_cfg_item.hpp file."""
    with open(hpp_file, 'r') as f:
        content = f.read()
    
    # Extract parameter definitions
    pattern = r'const\s+ubx_cfg_item_t\s+([A-Z0-9_]+)\s*=\s*\{\s*"([A-Z0-9_]+)"\s*,\s*(0x[0-9A-Fa-f]+)\s*,\s*([A-Z0-9]+)\s*,\s*([0-9.]+)\s*,\s*([A-Z]+)\s*\};'
    matches = re.findall(pattern, content)
    
    # Extract enum definitions
    enum_pattern = r'enum\s+([A-Z0-9_]+)_ENUM\s*\{([^}]+)\};'
    enum_matches = re.findall(enum_pattern, content)
    enum_values = {}
    
    for enum_name, enum_content in enum_matches:
        values = {}
        # Changed the regex to capture both numeric (e.g., '0') and hex (e.g., '0x01') values
        value_pattern = r'([A-Z0-9_]+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)'
        value_matches = re.findall(value_pattern, enum_content)
        for value_name, value in value_matches:
            # Ensure all values are in hex format with 0x prefix
            if value.startswith('0x'):
                hex_value = value
            else:
                # Convert decimal to hex with 0x prefix and at least 2 digits
                try:
                    int_val = int(value)
                    hex_value = f"0x{int_val:02x}"
                except ValueError:
                    hex_value = value  # Fallback if not a valid number
            
            # Store the value with the original enum identifier
            values[value_name] = hex_value
        enum_values[enum_name] = values
    
    parameters = []
    for match in matches:
        variable_name, name, key_id, type_name, scale, unit = match
        
        # Default applicable devices and firmware support
        # These would ideally be extracted from comments or other sources
        applicable_devices = ["ZED-F9P", "ZED-F9R"]
        
        firmware_support = {
            "ZED-F9P": {
                "since": "HPG 1.13"
            },
            "ZED-F9R": {
                "since": "HPS 1.13"
            }
        }
        
        # Extract the group from the name (e.g., CFG_INFMSG_UBX_USB -> INFMSG)
        parts = name.split('_')
        if len(parts) > 1:
            group = parts[1]
        else:
            group = "UNKNOWN"
        
        # Check if this parameter has an associated enum
        possible_values = None
        for enum_name, values in enum_values.items():
            if enum_name in variable_name:
                possible_values = values
                break
        
        parameter = {
            "name": name,
            "key_id": key_id,
            "type": type_name,
            "scale": float(scale),
            "unit": unit,
            "applicable_devices": applicable_devices,
            "description": f"{name} configuration parameter",  # Default description
            "group": group,
            "firmware_support": firmware_support
        }
        
        if possible_values:
            parameter["possible_values"] = possible_values
        
        parameters.append(parameter)
    
    return parameters

def create_toml_file(parameters: List[Dict[str, Any]], output_file: str) -> None:
    """Create a TOML file with the extracted parameters."""
    # Add firmware version information
    firmware_versions = {
        "ZED-F9P": [
            {
                "version": "HPG 1.13",
                "description": "Early production release",
                "release_date": "2020-06-15"
            },
            {
                "version": "HPG 1.30",
                "description": "Production release with improved RTK performance",
                "release_date": "2022-01-15"
            },
            {
                "version": "HPG 1.32",
                "description": "Latest production release with enhanced RTK performance",
                "release_date": "2024-01-14"
            }
        ],
        "ZED-F9R": [
            {
                "version": "HPS 1.13",
                "description": "Early production release",
                "release_date": "2020-08-10"
            },
            {
                "version": "HPS 1.20",
                "description": "Production release with improved dead reckoning",
                "release_date": "2022-03-10"
            },
            {
                "version": "HPS 1.30",
                "description": "Latest production release with enhanced dead reckoning",
                "release_date": "2024-01-14"
            }
        ]
    }
    
    data = {
        "version": "1.0.0",
        "device_types": ["ZED-F9P", "ZED-F9R"],
        "firmware_versions": firmware_versions,
        "parameters": parameters
    }
    
    # Write the TOML file
    try:
        with open(output_file, 'wb') as f:  # Use 'wb' mode for tomli_w
            tomli_w.dump(data, f)
        print(f"Extracted {len(parameters)} parameters to {output_file}")
    except Exception as e:
        print(f"Error writing TOML file: {e}")
        sys.exit(1)

def main():
    """Main function."""
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <ubx_cfg_item.hpp> <output.toml>")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    if not os.path.isfile(input_file):
        print(f"Error: Input file not found: {input_file}")
        sys.exit(1)
    
    parameters = extract_parameters(input_file)
    create_toml_file(parameters, output_file)

if __name__ == "__main__":
    main()
