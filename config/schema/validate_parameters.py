#!/usr/bin/env python3
"""
UBX-CFG Parameter Validator

This script validates UBX-CFG parameter definition files against the JSON schema.
It also provides additional validation checks for parameter consistency.

Usage:
  python validate_parameters.py <parameter_file.json>
"""

import json
import sys
import os
import jsonschema
from jsonschema import validate
from typing import Dict, List, Any, Set

def load_json_file(file_path: str) -> Dict:
    """Load and parse a JSON file."""
    try:
        with open(file_path, 'r') as f:
            return json.load(f)
    except json.JSONDecodeError as e:
        print(f"Error parsing JSON file: {e}")
        sys.exit(1)
    except FileNotFoundError:
        print(f"File not found: {file_path}")
        sys.exit(1)

def validate_against_schema(instance: Dict, schema: Dict) -> None:
    """Validate a JSON instance against a schema."""
    try:
        validate(instance=instance, schema=schema)
        print("✅ Schema validation passed")
    except jsonschema.exceptions.ValidationError as e:
        print(f"❌ Schema validation failed: {e}")
        sys.exit(1)

def check_key_id_uniqueness(parameters: List[Dict]) -> None:
    """Check that all key_ids are unique."""
    key_ids = {}
    for param in parameters:
        key_id = param['key_id']
        if key_id in key_ids:
            print(f"❌ Duplicate key_id found: {key_id} in parameters '{key_ids[key_id]}' and '{param['name']}'")
            sys.exit(1)
        key_ids[key_id] = param['name']
    print(f"✅ All {len(key_ids)} key_ids are unique")

def check_name_uniqueness(parameters: List[Dict]) -> None:
    """Check that all parameter names are unique."""
    names = {}
    for param in parameters:
        name = param['name']
        if name in names:
            print(f"❌ Duplicate parameter name found: {name}")
            sys.exit(1)
        names[name] = True
    print(f"✅ All {len(names)} parameter names are unique")

def check_device_type_consistency(parameters: List[Dict], allowed_device_types: List[str]) -> None:
    """Check that all applicable_devices entries are valid."""
    for param in parameters:
        for device in param['applicable_devices']:
            if device not in allowed_device_types:
                print(f"❌ Invalid device type '{device}' in parameter '{param['name']}'")
                print(f"   Allowed device types: {', '.join(allowed_device_types)}")
                sys.exit(1)
    print(f"✅ All device types are valid")

def check_enum_consistency(parameters: List[Dict]) -> None:
    """Check that E1 type parameters have possible_values defined."""
    for param in parameters:
        if param['type'] == 'E1' and 'possible_values' not in param:
            print(f"❌ Parameter '{param['name']}' has type E1 but no possible_values defined")
            sys.exit(1)
    print(f"✅ All E1 type parameters have possible_values defined")

def check_default_value_consistency(parameters: List[Dict]) -> None:
    """Check that default values are consistent with possible values for E1 types."""
    for param in parameters:
        if 'default_value' in param and param['type'] == 'E1' and 'possible_values' in param:
            default_value = param['default_value'].lower()
            possible_values = [v.lower() for v in param['possible_values'].values()]
            if default_value not in possible_values:
                print(f"❌ Default value '{param['default_value']}' for parameter '{param['name']}' is not in possible_values")
                print(f"   Possible values: {', '.join(param['possible_values'].values())}")
                sys.exit(1)
    print(f"✅ All default values are consistent with possible values")

def check_min_max_consistency(parameters: List[Dict]) -> None:
    """Check that min_value is less than max_value when both are defined."""
    for param in parameters:
        if 'min_value' in param and 'max_value' in param:
            min_val = int(param['min_value'], 16)
            max_val = int(param['max_value'], 16)
            if min_val >= max_val:
                print(f"❌ min_value ({param['min_value']}) is not less than max_value ({param['max_value']}) for parameter '{param['name']}'")
                sys.exit(1)
    print(f"✅ All min/max value pairs are consistent")

def main() -> None:
    """Main function."""
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <parameter_file.json>")
        sys.exit(1)

    param_file = sys.argv[1]
    
    # Determine the schema file path relative to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    schema_file = os.path.join(script_dir, "ubx_cfg_parameters_schema.json")
    
    print(f"Validating {param_file} against schema {schema_file}")
    
    schema = load_json_file(schema_file)
    params = load_json_file(param_file)
    
    # Validate against the schema
    validate_against_schema(params, schema)
    
    # Additional validation checks
    check_key_id_uniqueness(params['parameters'])
    check_name_uniqueness(params['parameters'])
    check_device_type_consistency(params['parameters'], params['device_types'])
    check_enum_consistency(params['parameters'])
    check_default_value_consistency(params['parameters'])
    check_min_max_consistency(params['parameters'])
    
    print("\n✅ All validation checks passed!")

if __name__ == "__main__":
    main()
