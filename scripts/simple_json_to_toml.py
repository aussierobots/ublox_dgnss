#!/usr/bin/env python3
"""
Simple script to convert UBX-CFG parameter files from JSON to TOML format.
This script doesn't require additional dependencies beyond Python's standard library.
"""

import json
import sys
import os
from collections import OrderedDict

def format_value(value):
    """Format a value for TOML output"""
    if isinstance(value, str):
        # For strings, keep 0x prefixed values as is, otherwise quote
        if value.startswith("0x") and all(c in "0123456789abcdefABCDEF" for c in value[2:]):
            return value
        return f'"{value}"'
    elif isinstance(value, list):
        # Format list as TOML array
        formatted_items = [format_value(item) for item in value]
        return "[" + ", ".join(formatted_items) + "]"
    elif isinstance(value, bool):
        return "true" if value else "false"
    elif value is None:
        return "null"
    else:
        return str(value)

def create_toml_from_json(data, indent=0):
    """Create TOML content from JSON data"""
    lines = []
    
    # Basic key-value pairs at the top level
    if indent == 0:
        lines.append("# UBX-CFG Full Parameters Configuration File")
        lines.append("# TOML version of the configuration (converted from JSON)")
        lines.append("")
        
        lines.append(f'version = "{data["version"]}"')
        device_types = ', '.join([f'"{device}"' for device in data["device_types"]])
        lines.append(f'device_types = [{device_types}]')
        lines.append("")
        
        # Handle firmware versions section
        lines.append("# Firmware version information")
        lines.append("[firmware_versions]")
        lines.append("")
        
        for device, versions in data["firmware_versions"].items():
            lines.append(f"# {device} firmware versions")
            for version in versions:
                lines.append(f'[[firmware_versions.{device}]]')
                lines.append(f'version = "{version["version"]}"')
                lines.append(f'description = "{version["description"]}"')
                lines.append(f'release_date = "{version["release_date"]}"')
                lines.append("")
        
        # Handle parameters section
        lines.append("# Parameters configuration")
        
        for param in data["parameters"]:
            lines.append(f"# Parameter: {param['name']}")
            lines.append("[[parameters]]")
            
            # Add basic properties
            for key in ["name", "key_id", "type", "scale", "unit", "description", "group", "default_value"]:
                if key in param:
                    if key in ["name", "key_id", "type", "unit", "description", "group"]:
                        lines.append(f'{key} = "{param[key]}"')
                    else:
                        lines.append(f'{key} = {param[key]}')
            
            # Add min/max values if present
            for key in ["min_value", "max_value"]:
                if key in param:
                    lines.append(f'{key} = "{param[key]}"')
            
            # Add notes if present
            if "notes" in param:
                lines.append(f'notes = "{param["notes"]}"')
            
            # Add applicable devices
            if "applicable_devices" in param:
                devices = ', '.join([f'"{device}"' for device in param["applicable_devices"]])
                lines.append(f'applicable_devices = [{devices}]')
            
            # Add possible values if present
            if "possible_values" in param:
                lines.append("[parameters.possible_values]")
                for key, value in param["possible_values"].items():
                    lines.append(f'{key} = "{value}"')
            
            # Add firmware support
            if "firmware_support" in param:
                for device, support in param["firmware_support"].items():
                    lines.append(f"[parameters.firmware_support.{device}]")
                    if "since" in support:
                        lines.append(f'since = "{support["since"]}"')
                    
                    # Add behavior changes if present
                    if "behavior_changes" in support:
                        for change in support["behavior_changes"]:
                            lines.append(f"[[parameters.firmware_support.{device}.behavior_changes]]")
                            lines.append(f'version = "{change["version"]}"')
                            lines.append(f'description = "{change["description"]}"')
            
            # Add empty line between parameters
            lines.append("")
    
    return "\n".join(lines)

def convert_json_to_toml(input_path, output_path):
    """
    Convert a JSON file to TOML format.
    
    Args:
        input_path: Path to the input JSON file
        output_path: Path to output the TOML file
    """
    print(f"Reading JSON from: {input_path}")
    try:
        with open(input_path, 'r') as f:
            data = json.load(f)
    except Exception as e:
        print(f"Error reading JSON file: {e}")
        return False
    
    # Generate TOML content
    toml_content = create_toml_from_json(data)
    
    print(f"Writing TOML to: {output_path}")
    try:
        with open(output_path, 'w') as f:
            f.write(toml_content)
        print("Conversion complete!")
        return True
    except Exception as e:
        print(f"Error writing TOML file: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) > 2:
        input_path = sys.argv[1]
        output_path = sys.argv[2]
    else:
        # Default paths
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_dir = os.path.dirname(script_dir)
        input_path = os.path.join(project_dir, "config", "ubx_cfg_parameters_full.json")
        output_path = os.path.join(project_dir, "config", "ubx_cfg_parameters_full.toml")
    
    success = convert_json_to_toml(input_path, output_path)
    sys.exit(0 if success else 1)
