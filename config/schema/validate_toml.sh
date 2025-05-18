#!/bin/bash
# Wrapper script to run the TOML validation script with the virtual environment

# Get the script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Activate the virtual environment
source "$SCRIPT_DIR/venv/bin/activate"

# Run the validation script with all arguments passed to this script
python "$SCRIPT_DIR/validate_toml_parameters.py" "$@"

# Deactivate the virtual environment
deactivate
