#!/bin/bash
# Wrapper script to run the TOML extraction script with the virtual environment

# Get the script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Activate the virtual environment
source "$SCRIPT_DIR/venv/bin/activate"

# Run the extraction script with all arguments passed to this script
python "$SCRIPT_DIR/extract_parameters_toml.py" "$@"

# Deactivate the virtual environment
deactivate
