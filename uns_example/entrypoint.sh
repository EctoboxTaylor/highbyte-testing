#!/bin/bash

# set errexit so that the script will exit on any error
set -e

# Run the python file
python3 "/app/scripts/dt_update.py"

# Keep the container running
tail -f /dev/null