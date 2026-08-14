#!/usr/bin/env bash
set -euo pipefail

readonly PACKAGE_NAME="dynamixel_body"
readonly RULE_BASENAME="99-dynamixel-body.rules"
readonly RULE_PATH="/etc/udev/rules.d/${RULE_BASENAME}"

device=""
symlink_name="dynamixel_body"
dry_run=false
remove_rule=false

usage() {
  cat <<'EOF'
Create a stable /dev/dynamixel_body symlink for the mannequin Dynamixel adapter.
The user running this installer is also added to the dialout group.

Usage:
  apply_dynamixel_body_udev.sh --device /dev/ttyUSBX [--symlink NAME] [--dry-run]
  apply_dynamixel_body_udev.sh --remove [--symlink NAME]

Options:
  --device PATH   Connected final adapter, preferably /dev/serial/by-id/...
  --symlink NAME  /dev symlink name (default: dynamixel_body)
  --dry-run       Print the detected identity and generated rule only
  --remove        Remove the installed generated rule
  -h, --help      Show this help

The adapter must expose a non-empty USB serial number. Matching only VID/PID
is intentionally refused because it could claim another Dynamixel adapter.
Stop dynamixel_body and Dynamixel Wizard before installing or removing a rule.
Log out and back in after the first install so the new dialout membership applies.
EOF
}

while (($#)); do
  case "$1" in
    --device)
      [[ $# -ge 2 ]] || { echo "ERROR: --device needs a path" >&2; exit 2; }
      device="$2"
      shift 2
      ;;
    --symlink)
      [[ $# -ge 2 ]] || { echo "ERROR: --symlink needs a name" >&2; exit 2; }
      symlink_name="$2"
      shift 2
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    --remove)
      remove_rule=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! "$symlink_name" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  echo "ERROR: invalid symlink name: ${symlink_name}" >&2
  exit 2
fi

admin=()
if ((EUID != 0)); then
  if ! command -v sudo >/dev/null 2>&1; then
    echo "ERROR: root privileges are required and sudo is unavailable" >&2
    exit 1
  fi
  admin=(sudo)
fi

# Preserve the interactive user when the entire script is invoked through sudo.
# A root-run provisioning job has no non-root user to grant access to.
access_user="${SUDO_USER:-$(id -un)}"
if [[ "$access_user" == "root" ]]; then
  access_user=""
fi

reload_rules() {
  "${admin[@]}" udevadm control --reload-rules
  udevadm settle
}

if "$remove_rule"; then
  if [[ -e "$RULE_PATH" ]]; then
    "${admin[@]}" unlink "$RULE_PATH"
    reload_rules
    echo "Removed ${RULE_PATH}"
  else
    echo "Rule is not installed: ${RULE_PATH}"
  fi
  if [[ -L "/dev/${symlink_name}" ]]; then
    "${admin[@]}" unlink "/dev/${symlink_name}"
    echo "Removed stale /dev/${symlink_name} symlink"
  fi
  exit 0
fi

if [[ -z "$device" ]]; then
  echo "ERROR: --device must identify the connected final adapter" >&2
  usage >&2
  exit 2
fi
if [[ ! -e "$device" ]]; then
  echo "ERROR: device does not exist: ${device}" >&2
  exit 1
fi
if ! command -v udevadm >/dev/null 2>&1; then
  echo "ERROR: udevadm is unavailable" >&2
  exit 1
fi

resolved_device="$(readlink -f "$device")"
if [[ ! "$resolved_device" =~ ^/dev/tty[A-Za-z0-9._-]+$ ]]; then
  echo "ERROR: resolved path is not a tty device: ${resolved_device}" >&2
  exit 1
fi

properties="$(udevadm info --query=property --name="$resolved_device")"
property_value() {
  local key="$1"
  printf '%s\n' "$properties" | sed -n "s/^${key}=//p" | head -n 1
}

vendor_id="$(property_value ID_VENDOR_ID)"
product_id="$(property_value ID_MODEL_ID)"
serial="$(property_value ID_SERIAL_SHORT)"

if [[ ! "$vendor_id" =~ ^[0-9A-Fa-f]{4}$ ]]; then
  echo "ERROR: no valid USB vendor ID found for ${resolved_device}" >&2
  exit 1
fi
if [[ ! "$product_id" =~ ^[0-9A-Fa-f]{4}$ ]]; then
  echo "ERROR: no valid USB product ID found for ${resolved_device}" >&2
  exit 1
fi
if [[ -z "$serial" || ! "$serial" =~ ^[A-Za-z0-9._:+-]+$ ]]; then
  echo "ERROR: the adapter has no safe unique USB serial number" >&2
  echo "Use an adapter with a programmed serial; VID/PID-only matching is refused." >&2
  exit 1
fi

package_dir="$(rospack find "$PACKAGE_NAME" 2>/dev/null || true)"
if [[ -n "$package_dir" ]]; then
  template="${package_dir}/udev/99-dynamixel-body.rules.template"
else
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  template="${script_dir}/../udev/99-dynamixel-body.rules.template"
fi
if [[ ! -f "$template" ]]; then
  echo "ERROR: udev rule template not found: ${template}" >&2
  exit 1
fi

temporary_rule="$(mktemp)"
trap 'unlink "$temporary_rule" 2>/dev/null || true' EXIT
sed \
  -e "s/@VENDOR_ID@/${vendor_id,,}/g" \
  -e "s/@PRODUCT_ID@/${product_id,,}/g" \
  -e "s/@SERIAL@/${serial}/g" \
  -e "s/@SYMLINK@/${symlink_name}/g" \
  "$template" > "$temporary_rule"

echo "Adapter identity:"
echo "  requested : ${device}"
echo "  resolved  : ${resolved_device}"
echo "  VID:PID   : ${vendor_id,,}:${product_id,,}"
echo "  serial    : ${serial}"
echo "  symlink   : /dev/${symlink_name}"
echo
echo "Generated rule:"
cat "$temporary_rule"

if "$dry_run"; then
  echo
  echo "Dry run only; no system file was changed."
  exit 0
fi

"${admin[@]}" install -o root -g root -m 0644 "$temporary_rule" "$RULE_PATH"

if [[ -n "$access_user" ]]; then
  if ! getent group dialout >/dev/null; then
    echo "ERROR: required group does not exist: dialout" >&2
    exit 1
  fi
  if ! id -nG "$access_user" | tr ' ' '\n' | grep -Fxq dialout; then
    "${admin[@]}" usermod -aG dialout "$access_user"
    echo "Added ${access_user} to the dialout group."
    echo "Log out and back in before starting dynamixel_body."
  else
    echo "User ${access_user} is already in the dialout group."
  fi
else
  echo "WARNING: installer was run as root; no non-root user was added to dialout." >&2
fi

reload_rules

tty_name="$(basename "$resolved_device")"
"${admin[@]}" udevadm trigger \
  --action=add \
  --subsystem-match=tty \
  --sysname-match="$tty_name"
udevadm settle

expected="$(readlink -f "$resolved_device")"
actual=""
if [[ -e "/dev/${symlink_name}" ]]; then
  actual="$(readlink -f "/dev/${symlink_name}")"
fi
if [[ "$actual" != "$expected" ]]; then
  echo "ERROR: installed ${RULE_PATH}, but /dev/${symlink_name} was not created for ${expected}" >&2
  echo "Unplug/replug the adapter, then run: udevadm settle" >&2
  exit 1
fi

echo
echo "Installed ${RULE_PATH}"
echo "Verified /dev/${symlink_name} -> ${actual}"
echo "Only one hardware process may open this adapter at a time."
