#!/usr/bin/env bash
# Build and install the out-of-tree Goodix SPI libfprint drivers.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ID="goodix-spi-driver"
OWNED_MARKER="# Managed by $PROJECT_ID; do not edit."
DEVICE_METADATA="$SCRIPT_DIR/config/supported-spi-devices.tsv"
LIBFPRINT_DIR="${LIBFPRINT_SOURCE_DIR:-}"
BUILDDIR="${BUILDDIR:-}"
PROJECT_INSTALL_ROOT="${PROJECT_INSTALL_ROOT:-/opt/$PROJECT_ID}"
INSTALL_DIR="${INSTALL_DIR:-$PROJECT_INSTALL_ROOT/libfprint}"
FPRINTD_INSTALL_DIR="${FPRINTD_INSTALL_DIR:-$PROJECT_INSTALL_ROOT/fprintd}"
STATE_DIR="${GOODIX_SPI_STATE_DIR:-/var/lib/$PROJECT_ID}"
MANIFEST_FILE="${GOODIX_SPI_INSTALL_MANIFEST:-$STATE_DIR/install.manifest}"
INSTALL_ADAPTIVE_FPRINTD="${INSTALL_ADAPTIVE_FPRINTD:-yes}"
FPRINTD_DROPIN_DIR="${FPRINTD_DROPIN_DIR:-/etc/systemd/system/fprintd.service.d}"
FPRINTD_DROPIN="${GOODIX_SPI_LIBFPRINT_DROPIN:-$FPRINTD_DROPIN_DIR/20-goodix-spi-libfprint.conf}"
FPRINTD_ADAPTIVE_DROPIN="${GOODIX_SPI_FPRINTD_DROPIN:-$FPRINTD_DROPIN_DIR/30-goodix-spi-adaptive.conf}"
UDEV_RULE_DST="${GOODIX_SPI_UDEV_RULE:-/etc/udev/rules.d/70-libfprint-2.rules}"
CHECK_ONLY=0
SKIP_HARDWARE_CHECK="${GOODIX_SPI_SKIP_HARDWARE_CHECK:-0}"
ENABLE_DEBUG="${GOODIX_SPI_DEBUG:-0}"
ENABLE_DEVELOPER="${GOODIX_SPI_DEVELOPER:-0}"
LIBFPRINT_PATH_SET=0
SUPPORTED_DRIVERS=()
SUPPORTED_ACPI_IDS=()
INSTALL_HOOKS=()
TEMP_FILES=()

usage() {
  printf 'Usage: %s [options] [path/to/libfprint]\n' "$0"
  printf '\nOptions:\n'
  printf '  --check                 run preflight checks without changing the system\n'
  printf '  --skip-hardware-check   allow packaging/building without attached hardware\n'
  printf '  --with-fprintd          install adaptive and warm-session fprintd (default)\n'
  printf '  --without-fprintd       keep the distro fprintd executable\n'
  printf '\nWithout a source path, the pinned upstream source is fetched into the cache.\n'
  printf 'Set GOODIX_SPI_DEBUG=1 only when verbose libfprint diagnostics are needed.\n'
  printf 'Set GOODIX_SPI_DEVELOPER=1 only for a developer build with capture/fault hooks.\n'
}

cleanup() {
  for file in "${TEMP_FILES[@]}"; do
    rm -f "$file"
  done
}
trap cleanup EXIT

run_root() {
  if [ "${EUID:-$(id -u)}" -eq 0 ]; then
    "$@"
  else
    sudo "$@"
  fi
}

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    printf 'Error: required command not found: %s\n' "$1" >&2
    return 1
  fi
}

append_unique() {
  local value="$1"
  shift
  local existing
  for existing in "$@"; do
    [ "$existing" = "$value" ] && return 1
  done
  return 0
}

load_device_metadata() {
  local driver acpi hook extra
  [ -f "$DEVICE_METADATA" ] || {
    printf 'Error: missing device metadata: %s\n' "$DEVICE_METADATA" >&2
    return 1
  }

  while read -r driver acpi hook extra; do
    [ -z "${driver:-}" ] && continue
    case "$driver" in \#*) continue ;; esac
    if [ -n "${extra:-}" ] || [ -z "${acpi:-}" ] || [ -z "${hook:-}" ]; then
      printf 'Error: malformed device metadata row for %s\n' "$driver" >&2
      return 1
    fi
    [[ "$driver" =~ ^[a-z0-9_]+$ ]] || {
      printf 'Error: invalid driver id in metadata: %s\n' "$driver" >&2
      return 1
    }
    [[ "$acpi" =~ ^[A-Z0-9]{8}$ ]] || {
      printf 'Error: invalid ACPI id in metadata: %s\n' "$acpi" >&2
      return 1
    }
    [ -d "$SCRIPT_DIR/drivers/$driver" ] || {
      printf 'Error: metadata driver directory is missing: drivers/%s\n' "$driver" >&2
      return 1
    }
    [ -x "$SCRIPT_DIR/config/install.d/$hook" ] || {
      printf 'Error: install hook is missing or not executable: config/install.d/%s\n' "$hook" >&2
      return 1
    }
    SUPPORTED_DRIVERS+=("$driver")
    SUPPORTED_ACPI_IDS+=("$acpi")
    if append_unique "$hook" "${INSTALL_HOOKS[@]}"; then
      INSTALL_HOOKS+=("$hook")
    fi
  done < "$DEVICE_METADATA"

  [ "${#SUPPORTED_DRIVERS[@]}" -gt 0 ] || {
    printf 'Error: no supported devices are declared.\n' >&2
    return 1
  }
}

# A LPSS SPI controller that found no DMA channel at probe (idma64 loaded after
# it) silently runs in PIO mode, and on some boards that truncates the last bytes
# of every sensor reply (see issue #8).  Warn, do not fail.
warn_if_spi_controller_uses_pio() {
  local device_path="$1" node controller pci chan in_use=0 seen=0
  node="$(readlink -f "$device_path")"
  controller="${node%%/spi_master/*}"
  case "$controller" in
    */pxa2xx-spi.*) ;;
    *) return 0 ;;
  esac
  pci="$(basename "$(dirname "$controller")")"
  for chan in /sys/class/dma/dma*chan*; do
    [ -e "$chan/device" ] || continue
    [ "$(basename "$(readlink -f "$chan/device")")" = "$pci" ] || continue
    seen=1
    if [ "$(cat "$chan/in_use" 2>/dev/null)" = "1" ]; then
      in_use=1
    fi
  done
  if [ "$seen" -eq 1 ] && [ "$in_use" -eq 0 ]; then
    cat >&2 <<EOF_PIO
Warning: the SPI controller behind $device_path ($(basename "$controller"), PCI $pci)
holds no DMA channel, so it runs in PIO mode. On some boards this truncates the
last bytes of every sensor reply (empty chip-id/OTP responses, TLS record
errors). Usually idma64 was loaded after the SPI controller; load it first:
  echo 'softdep spi_pxa2xx_platform pre: idma64' | sudo tee /etc/modprobe.d/goodix-spi-dma.conf
  dracut:          echo 'add_drivers+=" idma64 "' | sudo tee /etc/dracut.conf.d/goodix-spi-dma.conf; sudo dracut -f
  initramfs-tools: echo idma64 | sudo tee -a /etc/initramfs-tools/modules; sudo update-initramfs -u
Reboot, then check:  journalctl -k -b | grep -i 'using PIO' || echo 'DMA OK'
EOF_PIO
  fi
}

check_hardware() {
  local acpi path found=0
  if [ "$SKIP_HARDWARE_CHECK" = "1" ]; then
    printf 'Hardware check: skipped explicitly.\n'
    return 0
  fi

  for acpi in "${SUPPORTED_ACPI_IDS[@]}"; do
    for path in /sys/bus/spi/devices/spi-"$acpi":*; do
      if [ -e "$path" ]; then
        printf 'Hardware check: found %s at %s\n' "$acpi" "$path"
        found=1
        warn_if_spi_controller_uses_pio "$path"
      fi
    done
  done

  if [ "$found" -ne 1 ]; then
    printf 'Error: no supported Goodix SPI ACPI device was found. Supported IDs: %s\n' \
      "${SUPPORTED_ACPI_IDS[*]}" >&2
    printf 'Use --skip-hardware-check only for CI or offline packaging.\n' >&2
    return 1
  fi
}

preflight() {
  local command driver acpi hook package failed=0
  local packages=(glib-2.0 gio-2.0 gusb gudev-1.0 libgpiod openssl)
  for command in bash cc git install meson ninja pkg-config; do
    require_command "$command" || failed=1
  done
  if [ "${EUID:-$(id -u)}" -ne 0 ]; then
    require_command sudo || failed=1
  fi
  [ "$failed" -eq 0 ] || return 1

  case "$ENABLE_DEBUG" in
    0|1) ;;
    *)
      printf 'Error: GOODIX_SPI_DEBUG must be 0 or 1.\n' >&2
      return 1
      ;;
  esac

  case "$ENABLE_DEVELOPER" in
    0|1) ;;
    *)
      printf 'Error: GOODIX_SPI_DEVELOPER must be 0 or 1.\n' >&2
      return 1
      ;;
  esac

  case "$INSTALL_ADAPTIVE_FPRINTD" in
    yes|1|true|ask|auto) packages+=(polkit-gobject-1) ;;
  esac
  for package in "${packages[@]}"; do
    if ! pkg-config --exists "$package"; then
      printf 'Error: required development package is unavailable to pkg-config: %s\n' "$package" >&2
      failed=1
    fi
  done
  [ "$failed" -eq 0 ] || return 1

  for fragment in \
    integration/libfprint-project/meson.build \
    integration/libfprint-project/meson_options.txt \
    integration/libfprint-drivers/meson.build \
    integration/libfprint-tests/meson.build; do
    [ -f "$SCRIPT_DIR/$fragment" ] || {
      printf 'Error: missing integration fragment: %s\n' "$fragment" >&2
      return 1
    }
  done

  for driver in "${SUPPORTED_DRIVERS[@]}"; do
    grep -Fq "'$driver'" "$SCRIPT_DIR/integration/libfprint-project/meson.build" || {
      printf 'Error: driver %s is missing from project Meson registration.\n' "$driver" >&2
      return 1
    }
    grep -Fq "'$driver'" "$SCRIPT_DIR/integration/libfprint-drivers/meson.build" || {
      printf 'Error: driver %s is missing from source Meson registration.\n' "$driver" >&2
      return 1
    }
  done
  for acpi in "${SUPPORTED_ACPI_IDS[@]}"; do
    grep -R -Fq --include='*.c' --include='*.h' "$acpi" "$SCRIPT_DIR/drivers" || {
      printf 'Error: ACPI id %s is declared but absent from driver sources.\n' "$acpi" >&2
      return 1
    }
  done

  check_hardware
  for hook in "${INSTALL_HOOKS[@]}"; do
    "$SCRIPT_DIR/config/install.d/$hook" check
  done
  check_owned_destination "$FPRINTD_DROPIN"
  check_owned_destination "$UDEV_RULE_DST"
  check_owned_destination "$MANIFEST_FILE"
  printf 'Installer preflight: OK (%s)\n' "${SUPPORTED_DRIVERS[*]}"
}

check_owned_destination() {
  local path="$1" first
  if [ -e "$path" ]; then
    if [ -r "$path" ]; then
      first="$(head -n 1 "$path" 2>/dev/null || true)"
    else
      first="$(run_root head -n 1 "$path" 2>/dev/null || true)"
    fi
    if [ "$first" != "$OWNED_MARKER" ]; then
      printf 'Error: refusing to overwrite non-project file: %s\n' "$path" >&2
      printf 'Move it aside or choose an override path explicitly.\n' >&2
      return 1
    fi
  fi
}

install_owned_file() {
  local source="$1" destination="$2" mode="$3" temp
  check_owned_destination "$destination"
  temp="$(mktemp)"
  TEMP_FILES+=("$temp")
  {
    printf '%s\n' "$OWNED_MARKER"
    cat "$source"
  } > "$temp"
  run_root install -d "$(dirname "$destination")"
  run_root install -m "$mode" "$temp" "$destination"
}

seed_device_state() {
  local hook temp destination state_name rc
  for hook in "${INSTALL_HOOKS[@]}"; do
    temp="$(mktemp)"
    TEMP_FILES+=("$temp")
    set +e
    destination="$("$SCRIPT_DIR/config/install.d/$hook" seed-state "$temp")"
    rc=$?
    set -e
    if [ "$rc" -eq 3 ]; then
      continue
    fi
    [ "$rc" -eq 0 ] || {
      printf 'Error: state seeding failed for install hook %s.\n' "$hook" >&2
      return "$rc"
    }
    state_name="${destination#/var/lib/fprint/}"
    if [ "$state_name" = "$destination" ] ||
       [[ ! "$state_name" =~ ^[A-Za-z0-9._-]+$ ]] ||
       [ "$state_name" = "." ] || [ "$state_name" = ".." ]; then
      printf 'Error: unsafe state destination from %s: %s\n' \
        "$hook" "$destination" >&2
      return 1
    fi
    [ -s "$temp" ] || {
      printf 'Error: %s returned empty state for %s.\n' "$hook" "$destination" >&2
      return 1
    }
    run_root install -d -m 0700 "$(dirname "$destination")"
    run_root install -m 0600 "$temp" "$destination"
    printf 'Seeded protected device state at %s\n' "$destination"
  done
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --check) CHECK_ONLY=1 ;;
    --skip-hardware-check) SKIP_HARDWARE_CHECK=1 ;;
    --with-fprintd) INSTALL_ADAPTIVE_FPRINTD=yes ;;
    --without-fprintd) INSTALL_ADAPTIVE_FPRINTD=no ;;
    --)
      shift
      [ "$#" -le 1 ] || { usage >&2; exit 2; }
      if [ "$#" -eq 1 ]; then LIBFPRINT_DIR="$1"; LIBFPRINT_PATH_SET=1; fi
      break
      ;;
    -*) printf 'Error: unknown option %s\n' "$1" >&2; usage >&2; exit 2 ;;
    *)
      [ "$LIBFPRINT_PATH_SET" -eq 0 ] || {
        printf 'Error: only one libfprint source path may be supplied.\n' >&2
        exit 2
      }
      LIBFPRINT_DIR="$1"
      LIBFPRINT_PATH_SET=1
      ;;
  esac
  shift
done

load_device_metadata
preflight
if [ "$CHECK_ONLY" -eq 1 ]; then
  exit 0
fi

case "$INSTALL_ADAPTIVE_FPRINTD" in
  yes|1|true) INSTALL_ADAPTIVE_FPRINTD=1 ;;
  no|0|false) INSTALL_ADAPTIVE_FPRINTD=0 ;;
  ask|auto)
    if [ -t 0 ] && [ -t 1 ]; then
      printf 'Replace fprintd with the adaptive and warm-session build? [Y/n] '
      read -r reply
      case "$reply" in n|N|no|NO|No) INSTALL_ADAPTIVE_FPRINTD=0 ;; *) INSTALL_ADAPTIVE_FPRINTD=1 ;; esac
    else
      INSTALL_ADAPTIVE_FPRINTD=1
      printf 'Non-interactive install: installing adaptive and warm-session fprintd by default.\n' >&2
    fi
    ;;
  *) printf 'Error: INSTALL_ADAPTIVE_FPRINTD must be ask, yes, or no.\n' >&2; exit 2 ;;
esac

if [ "$INSTALL_ADAPTIVE_FPRINTD" -eq 1 ]; then
  check_owned_destination "$FPRINTD_ADAPTIVE_DROPIN"
fi

if [ -z "$LIBFPRINT_DIR" ]; then
  LIBFPRINT_DIR="$("$SCRIPT_DIR/scripts/fetch-libfprint.sh")"
fi
if [ -z "$BUILDDIR" ]; then
  BUILDDIR="$LIBFPRINT_DIR/builddir"
fi
if [ ! -f "$LIBFPRINT_DIR/meson.build" ] || [ ! -f "$LIBFPRINT_DIR/libfprint/meson.build" ]; then
  printf 'Error: %s does not look like a libfprint source tree.\n' "$LIBFPRINT_DIR" >&2
  exit 1
fi

"$SCRIPT_DIR/scripts/libfprint/build.sh" --profile install \
  "$LIBFPRINT_DIR" "$BUILDDIR"

BUILT_LIB="$BUILDDIR/libfprint/libfprint-2.so.2.0.0"
BUILT_UDEV_RULE="$BUILDDIR/libfprint/70-libfprint-2.rules"
[ -f "$BUILT_LIB" ] || { printf 'Error: built libfprint was not found.\n' >&2; exit 1; }
[ -f "$BUILT_UDEV_RULE" ] || { printf 'Error: generated libfprint udev rules were not found.\n' >&2; exit 1; }
# Accept the already-widened form too: the sed below edits the build output in
# place, so a re-run without a libfprint rebuild sees acpi:ID:* here.
for acpi in "${SUPPORTED_ACPI_IDS[@]}"; do
  grep -Fq -e "ENV{MODALIAS}==\"acpi:$acpi:\"" -e "ENV{MODALIAS}==\"acpi:$acpi:*\"" "$BUILT_UDEV_RULE" || {
    printf 'Error: generated udev rules do not contain supported ACPI id %s.\n' "$acpi" >&2
    exit 1
  }
done

# libfprint's generator emits an exact MODALIAS match (acpi:ID:). Firmware that
# also advertises a _CID presents the device as acpi:ID:ID:, which the exact
# match misses -- leaving spidev unbound and the sensor invisible until a manual
# bind. Widen each supported id to a glob so the rule fires with or without the
# trailing _CID. The exact form was just validated to be present.
for acpi in "${SUPPORTED_ACPI_IDS[@]}"; do
  sed -i "s#ENV{MODALIAS}==\"acpi:${acpi}:\"#ENV{MODALIAS}==\"acpi:${acpi}:*\"#g" \
    "$BUILT_UDEV_RULE"
done

# Recheck immediately before the first system write. The preflight check makes
# --check useful, while this closes the gap if a destination changed mid-build.
check_owned_destination "$FPRINTD_DROPIN"
check_owned_destination "$UDEV_RULE_DST"
check_owned_destination "$MANIFEST_FILE"

run_root install -d "$INSTALL_DIR" "$FPRINTD_DROPIN_DIR"
run_root install -m 0755 "$BUILT_LIB" "$INSTALL_DIR/libfprint-2.so.2.0.0"
run_root ln -sfn libfprint-2.so.2.0.0 "$INSTALL_DIR/libfprint-2.so.2"
run_root ln -sfn libfprint-2.so.2.0.0 "$INSTALL_DIR/libfprint-2.so"

DROPIN_TMP="$(mktemp)"
TEMP_FILES+=("$DROPIN_TMP")
{
  printf '[Service]\n'
  printf 'Environment=LD_LIBRARY_PATH=%s\n' "$INSTALL_DIR"
  if [ "$ENABLE_DEBUG" = "1" ]; then
    printf 'Environment=G_MESSAGES_DEBUG=all\n'
  fi
  for hook in "${INSTALL_HOOKS[@]}"; do
    "$SCRIPT_DIR/config/install.d/$hook" emit
  done
} > "$DROPIN_TMP"
install_owned_file "$DROPIN_TMP" "$FPRINTD_DROPIN" 0644
install_owned_file "$BUILT_UDEV_RULE" "$UDEV_RULE_DST" 0644

if [ "$INSTALL_ADAPTIVE_FPRINTD" -eq 1 ]; then
  FPRINTD_RESTART=0 \
  LIBFPRINT_BUILD="$BUILDDIR" \
  LIBFPRINT_INSTALL_DIR="$INSTALL_DIR" \
  FPRINTD_INSTALL_DIR="$FPRINTD_INSTALL_DIR" \
  FPRINTD_DROPIN_DIR="$FPRINTD_DROPIN_DIR" \
  FPRINTD_DROPIN="$FPRINTD_ADAPTIVE_DROPIN" \
    "$SCRIPT_DIR/scripts/build-install-fprintd.sh"
else
  if run_root test -e "$FPRINTD_ADAPTIVE_DROPIN" &&
     [ "$(run_root head -n 1 "$FPRINTD_ADAPTIVE_DROPIN" 2>/dev/null || true)" = "$OWNED_MARKER" ]; then
    run_root rm -f "$FPRINTD_ADAPTIVE_DROPIN" "$FPRINTD_INSTALL_DIR/fprintd"
    run_root rmdir "$FPRINTD_INSTALL_DIR" 2>/dev/null || true
  fi
fi

# Seed an explicitly supplied key only after every build and executable/unit
# installation has succeeded, immediately before recording and activating the
# new installation.
seed_device_state

MANIFEST_TMP="$(mktemp)"
TEMP_FILES+=("$MANIFEST_TMP")
{
  printf '%s\n' "$OWNED_MARKER"
  printf '%s\n' \
    "$INSTALL_DIR/libfprint-2.so.2.0.0" \
    "$INSTALL_DIR/libfprint-2.so.2" \
    "$INSTALL_DIR/libfprint-2.so" \
    "$FPRINTD_DROPIN" \
    "$UDEV_RULE_DST"
  if [ "$INSTALL_ADAPTIVE_FPRINTD" -eq 1 ]; then
    printf '%s\n' "$FPRINTD_INSTALL_DIR/fprintd" "$FPRINTD_ADAPTIVE_DROPIN"
  fi
} > "$MANIFEST_TMP"
run_root install -d "$STATE_DIR"
run_root install -m 0644 "$MANIFEST_TMP" "$MANIFEST_FILE"

if command -v udevadm >/dev/null 2>&1; then
  run_root udevadm control --reload-rules
  run_root udevadm trigger --subsystem-match=spi --action=change
  run_root udevadm settle
fi
if command -v systemctl >/dev/null 2>&1; then
  run_root systemctl daemon-reload
  run_root systemctl restart fprintd.service
fi

printf '\nInstalled Goodix SPI drivers: %s\n' "${SUPPORTED_DRIVERS[*]}"
printf 'Test with: fprintd-list "$USER" && fprintd-verify\n'
