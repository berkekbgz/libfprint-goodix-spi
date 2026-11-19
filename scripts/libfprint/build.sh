#!/usr/bin/env bash
# The single libfprint build implementation used by installation and CI.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
DEVICE_METADATA="$REPO_DIR/config/supported-spi-devices.tsv"
PROFILE=install
DEVELOPER="${GOODIX_SPI_DEVELOPER:-0}"

usage() {
  printf 'Usage: %s [--profile install|ci] SOURCE_DIR BUILD_DIR\n' "$0"
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --profile)
      [ "$#" -ge 2 ] || { usage >&2; exit 2; }
      PROFILE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    -*)
      printf 'Error: unknown option %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
    *)
      break
      ;;
  esac
done

[ "$#" -eq 2 ] || { usage >&2; exit 2; }
SOURCE_DIR="$1"
BUILD_DIR="$2"

case "$PROFILE" in
  install|ci) ;;
  *)
    printf 'Error: unknown libfprint build profile: %s\n' "$PROFILE" >&2
    exit 2
    ;;
esac

case "$DEVELOPER" in
  0) developer_option=false ;;
  1) developer_option=true ;;
  *)
    printf 'Error: GOODIX_SPI_DEVELOPER must be 0 or 1.\n' >&2
    exit 2
    ;;
esac

"$SCRIPT_DIR/prepare-tree.sh" "$SOURCE_DIR"

meson_options=(-Dgoodix_spi_developer="$developer_option")
if [ "$PROFILE" = "ci" ]; then
  mapfile -t supported_drivers < <(
    awk '!/^#/ && NF && !seen[$1]++ { print $1 }' "$DEVICE_METADATA"
  )
  [ "${#supported_drivers[@]}" -gt 0 ] || {
    printf 'Error: no supported drivers are declared.\n' >&2
    exit 1
  }
  driver_list="$(IFS=,; printf '%s' "${supported_drivers[*]}")"
  meson_options+=(
    -Ddrivers="$driver_list"
    -Ddoc=false
    -Dintrospection=false
    -Dinstalled-tests=false
    -Dgtk-examples=false
    -Dudev_rules=disabled
    -Dudev_hwdb=disabled
  )
fi

if [ -f "$BUILD_DIR/meson-private/coredata.dat" ]; then
  meson setup "$BUILD_DIR" "$SOURCE_DIR" --reconfigure "${meson_options[@]}"
else
  if [ -e "$BUILD_DIR" ]; then
    stale_build="$BUILD_DIR.stale.$(date +%Y%m%d%H%M%S)"
    mv "$BUILD_DIR" "$stale_build"
    printf 'Moved non-Meson build path to %s\n' "$stale_build"
  fi
  meson setup "$BUILD_DIR" "$SOURCE_DIR" "${meson_options[@]}"
fi
meson compile -C "$BUILD_DIR"

printf 'Built libfprint with the %s profile in %s\n' "$PROFILE" "$BUILD_DIR"
if [ "$DEVELOPER" = "1" ]; then
  printf 'WARNING: developer-only capture diagnostics and fault injection are compiled in.\n' >&2
fi
