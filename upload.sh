#!/usr/bin/env bash
set -e

#
# Upload Characterization Firmware to Arduino Nano Every
#
TARGET_DIR="firmware/piano_firmware"
CLI="/opt/homebrew/bin/arduino-cli"
if [ ! -f "$CLI" ]; then
    CLI="arduino-cli"
fi

FQBN="arduino:megaavr:nona4809"

echo "============================================================"
echo "Target: Characterization Firmware ($TARGET_DIR)"
echo "Board:  Arduino Nano Every ($FQBN)"
echo "============================================================"

# Auto-detect serial port
PORT=""
if [[ "$OSTYPE" == "darwin"* ]]; then
    PORTS=(/dev/cu.usbmodem*)
    if [ -e "${PORTS[0]}" ]; then
        PORT="${PORTS[0]}"
    fi
else
    PORTS=(/dev/ttyACM*)
    if [ -e "${PORTS[0]}" ]; then
        PORT="${PORTS[0]}"
    fi
fi

if [ -z "$PORT" ]; then
    echo "No Arduino Nano Every detected!"
    echo "1. Connect your Nano Every via USB."
    echo "2. Double-tap the reset button if the port does not appear."
    exit 1
fi

echo "Found board on: $PORT"
echo ""
echo "Step 1: Compiling..."
"$CLI" compile --fqbn "$FQBN" "$TARGET_DIR"

echo "Step 2: Uploading to $PORT..."
"$CLI" upload -p "$PORT" --fqbn "$FQBN" "$TARGET_DIR"

echo ""
echo "Upload complete! Firmware is running on your Nano Every."
echo "============================================================"
echo "To test:"
echo "  python3 firmware/characterize/characterize.py check"
echo "  python3 firmware/characterize/characterize.py hit 22 12"
echo "============================================================"
