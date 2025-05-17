#!/usr/bin/env python3
"""
Script to convert UBX-CFG parameter files from JSON to TOML format.
This script preserves the structure and hierarchical relationships of the original JSON.
"""

import json
import sys
import os

try:
    import tomlkit
except ImportError:
    print("tomlkit package not found. Installing...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "tomlkit"])
    import tomlkit

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
    
    # Create TOML document
    doc = tomlkit.document()
    doc.add(tomlkit.comment("UBX-CFG Full Parameters Configuration File"))
    doc.add(tomlkit.comment("TOML version of the configuration (converted from JSON)"))
    doc.add(tomlkit.nl())
    
    # Add basic version information
    doc["version"] = data["version"]
    doc["device_types"] = data["device_types"]
    doc.add(tomlkit.nl())
    
    # Add firmware version information
    doc.add(tomlkit.comment("Firmware version information"))
    firmware_versions = tomlkit.table()
    doc["firmware_versions"] = firmware_versions
    
    # Process firmware versions for each device type
    for device_type, versions in data["firmware_versions"].items():
        doc.add(tomlkit.comment(f"{device_type} firmware versions"))
        firmware_versions[device_type] = tomlkit.array(True)
        
        for version_data in versions:
            # Create version table
            version_table = tomlkit.table()
            version_table["version"] = version_data["version"]
            version_table["description"] = version_data["description"]
            version_table["release_date"] = version_data["release_date"]
            
            # Add to the array
            firmware_versions[device_type].append(version_table)
    
    # Process parameters
    doc.add(tomlkit.nl())
    doc.add(tomlkit.comment("Parameters configuration"))
    parameters_array = tomlkit.array(True)  # Array of tables
    
    # Add each parameter
    for param in data["parameters"]:
        # Add comment for parameter name
        doc.add(tomlkit.comment(f"Parameter: {param['name']}"))
        
        # Create parameter table
        parameter = tomlkit.table()
        
        # Add simple fields
        for key in ["name", "key_id", "type", "scale", "unit", "description", "default_value", "group"]:
            if key in param:
                parameter[key] = param[key]
        
        # Add min/max values if present
        for key in ["min_value", "max_value"]:
            if key in param:
                parameter[key] = param[key]
                
        # Add notes if present
        if "notes" in param:
            parameter["notes"] = param["notes"]
        
        # Add applicable devices as array
        if "applicable_devices" in param:
            parameter["applicable_devices"] = param["applicable_devices"]
        
        # Add possible values if present
        if "possible_values" in param:
            possible_values = tomlkit.table()
            for key, value in param["possible_values"].items():
                possible_values[key] = value
            parameter["possible_values"] = possible_values
        
        # Add firmware support
        if "firmware_support" in param:
            firmware_support = tomlkit.table()
            for device, support in param["firmware_support"].items():
                device_support = tomlkit.table()
                if "since" in support:
                    device_support["since"] = support["since"]
                
                # Add behavior changes if present
                if "behavior_changes" in support:
                    behavior_changes = tomlkit.array(True)
                    for change in support["behavior_changes"]:
                        change_table = tomlkit.table()
                        change_table["version"] = change["version"]
                        change_table["description"] = change["description"]
                        behavior_changes.append(change_table)
                    device_support["behavior_changes"] = behavior_changes
                
                firmware_support[device] = device_support
            parameter["firmware_support"] = firmware_support
        
        # Add parameter to array
        parameters_array.append(parameter)
    
    # Add parameters array to document
    doc["parameters"] = parameters_array
    
    print(f"Writing TOML to: {output_path}")
    try:
        with open(output_path, 'w') as f:
            f.write(tomlkit.dumps(doc))
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
