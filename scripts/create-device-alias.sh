#!/usr/bin/env bash
set -euo pipefail

# Usage: ./create-device-alias.sh <device> <alias>
# Creates a permanent /dev/<alias> symlink for the given device.
# Example: ./create-device-alias.sh /dev/ttyUSB0 imu
#
# Re-running with the same alias will replace the existing rule.

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 <device> <alias> (e.g. $0 /dev/ttyUSB0 imu)"
  exit 1
fi

DEV="$1"
ALIAS="$2"
RULES_FILE="/etc/udev/rules.d/99-${ALIAS}.rules"

if [[ ! -e "$DEV" ]]; then
  echo "ERROR: Device $DEV not found."
  exit 1
fi

if [[ -f "$RULES_FILE" ]]; then
  echo "Replacing existing rule at $RULES_FILE:"
  echo "  $(cat "$RULES_FILE")"
fi

info=$(udevadm info -a -n "$DEV" 2>/dev/null)

extract() {
  echo "$info" | grep "ATTRS{$1}" | head -1 | sed 's/.*=="\(.*\)"/\1/'
}

VENDOR=$(extract idVendor)
PRODUCT=$(extract idProduct)
SERIAL=$(extract serial)

if [[ -z "$VENDOR" || -z "$PRODUCT" ]]; then
  echo "ERROR: Could not read idVendor/idProduct from $DEV."
  exit 1
fi

echo "Device:  $DEV"
echo "Alias:   /dev/$ALIAS"
echo "Vendor:  $VENDOR"
echo "Product: $PRODUCT"
echo "Serial:  ${SERIAL:-"(none)"}"

UDEV_RULE="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$VENDOR\", ATTRS{idProduct}==\"$PRODUCT\", ATTRS{serial}==\"$SERIAL\", SYMLINK+=\"$ALIAS\", MODE=\"0666\""

echo "$UDEV_RULE" | sudo tee "$RULES_FILE" > /dev/null

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Udev rule installed at $RULES_FILE. Reconnect the device if it is already plugged in."
