#!/bin/bash
#
# led_script.sh – run Python LED program inside venv
#

# Path to your virtual environment
VENV_PATH="/home/da-pi/led_strip/venv"

# Path to your Python script
PY_SCRIPT="/home/da-pi/led_strip/8pxl_off.py"

# Log file
LOG_FILE="/home/da-pi/led_script.log"

echo "[$(date)] Starting LED program..." >> "$LOG_FILE"

# Activate venv and run script
source "$VENV_PATH/bin/activate"
python "$PY_SCRIPT"

echo "[$(date)] LED program finished." >> "$LOG_FILE"

