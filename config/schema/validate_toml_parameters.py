#!/usr/bin/env python3
"""
Validation script for UBX-CFG parameter files in TOML format.

This script validates TOML parameter definition files against a schema
defined using Pydantic models. It provides detailed error messages for
validation failures.

Usage:
  python validate_toml_parameters.py <parameter_file.toml>
"""

import sys
import os
from pathlib import Path
from typing import List, Dict, Optional, Any, Union
import re
try:
    import tomllib  # Python 3.11+
except ImportError:
    import tomli as tomllib  # Python 3.7-3.10

from pydantic import BaseModel, Field, ValidationError, field_validator, model_validator

# Define Pydantic models for validation

class BehaviorChange(BaseModel):
    """Model for firmware behavior changes."""
    version: str
    description: str

class FirmwareSupport(BaseModel):
    """Model for firmware support information."""
    since: str
    until: Optional[str] = None
    behavior_changes: Optional[List[BehaviorChange]] = None

class FirmwareVersion(BaseModel):
    """Model for firmware version information."""
    version: str
    description: str
    release_date: str

class UbxCfgParameter(BaseModel):
    """Model for a UBX-CFG parameter."""
    name: str
    key_id: str
    type: str
    scale: float
    unit: str
    applicable_devices: List[str]
    description: str
    group: str
    firmware_support: Dict[str, FirmwareSupport]
    possible_values: Optional[Dict[str, str]] = None
    default_value: Optional[Union[str, int, float]] = None

    @field_validator('key_id')
    def validate_key_id(cls, v):
        """Validate key_id format (hexadecimal)."""
        if not re.match(r'^0x[0-9A-Fa-f]+$', v):
            raise ValueError(f'key_id must be in hexadecimal format (0x...): {v}')
        return v
    
    @field_validator('type')
    def validate_type(cls, v):
        """Validate parameter type."""
        valid_types = ['U1', 'U2', 'U4', 'U8', 'I1', 'I2', 'I4', 'I8', 'X1', 'X2', 'X4', 'X8', 'E1', 'E2', 'E4', 'L', 'R4', 'R8']
        if v not in valid_types:
            raise ValueError(f'type must be one of {valid_types}: {v}')
        return v
    
    @field_validator('applicable_devices')
    def validate_applicable_devices(cls, v):
        """Validate applicable_devices is not empty."""
        if not v:
            raise ValueError('applicable_devices cannot be empty')
        return v
    
    @field_validator('possible_values')
    def validate_possible_values(cls, v):
        """Validate possible_values format if present."""
        if v is not None:
            for key, value in v.items():
                if not re.match(r'^[A-Za-z0-9_]+$', key):
                    raise ValueError(f'possible_values key must be alphanumeric: {key}')
                if not re.match(r'^0x[0-9A-Fa-f]+$|^\d+$', value):
                    raise ValueError(f'possible_values value must be a number or hex: {value}')
        return v

class UbxCfgParameterFile(BaseModel):
    """Model for the entire parameter file."""
    version: str
    device_types: List[str]
    firmware_versions: Dict[str, List[FirmwareVersion]]
    parameters: List[UbxCfgParameter]

    @model_validator(mode='after')
    def validate_device_references(self):
        """Validate that all referenced devices exist in device_types."""
        if not hasattr(self, 'device_types') or not hasattr(self, 'parameters'):
            return self
            
        device_types = self.device_types
        parameters = self.parameters
        
        for i, param in enumerate(parameters):
            for device in param.applicable_devices:
                if device not in device_types:
                    raise ValueError(f'Parameter {param.name} references unknown device type: {device}')
            
            for device in param.firmware_support:
                if device not in device_types:
                    raise ValueError(f'Parameter {param.name} has firmware support for unknown device type: {device}')
        
        return self

def load_toml_file(file_path: str) -> Dict:
    """Load and parse a TOML file."""
    try:
        with open(file_path, "rb") as f:
            return tomllib.load(f)
    except Exception as e:
        print(f"Error parsing TOML file: {e}")
        sys.exit(1)

def validate_toml_parameters(file_path: str) -> bool:
    """Validate a TOML parameter file against the Pydantic schema."""
    print(f"Validating TOML parameter file: {file_path}")
    
    # Load the TOML file
    data = load_toml_file(file_path)
    
    try:
        # Validate against our Pydantic model
        UbxCfgParameterFile(**data)
        print("✅ Validation successful! The TOML file conforms to the schema.")
        return True
    except ValidationError as e:
        print("❌ Validation failed with the following errors:")
        for error in e.errors():
            # Format the error location
            location = ".".join(str(loc) for loc in error['loc'])
            print(f"  • {location}: {error['msg']}")
        return False

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <parameter_file.toml>")
        sys.exit(1)

    param_file = sys.argv[1]
    if not os.path.isfile(param_file):
        print(f"Error: File not found: {param_file}")
        sys.exit(1)

    success = validate_toml_parameters(param_file)
    sys.exit(0 if success else 1)
