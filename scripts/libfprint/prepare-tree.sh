#!/usr/bin/env bash
# Synchronize this project into an upstream libfprint tree and apply integration.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
DEVICE_METADATA="$REPO_DIR/config/supported-spi-devices.tsv"
PATCH_FILE="$REPO_DIR/patches/libfprint/1.94.10.patch"
OPTIONS_FRAGMENT="$REPO_DIR/integration/libfprint-project/meson_options.txt"
SOURCE_DIR="${1:-}"

usage() {
  printf 'Usage: %s /path/to/libfprint-source\n' "$0"
}

if [ "$#" -ne 1 ] ||
   [ ! -f "$SOURCE_DIR/meson.build" ] ||
   [ ! -f "$SOURCE_DIR/libfprint/meson.build" ]; then
  usage >&2
  exit 2
fi

mapfile -t supported_drivers < <(
  awk '!/^#/ && NF && !seen[$1]++ { print $1 }' "$DEVICE_METADATA"
)
[ "${#supported_drivers[@]}" -gt 0 ] || {
  printf 'Error: no supported drivers are declared in %s.\n' "$DEVICE_METADATA" >&2
  exit 1
}

printf 'Synchronizing Goodix SPI sources into %s\n' "$SOURCE_DIR"
for driver in "${supported_drivers[@]}"; do
  [ -d "$REPO_DIR/drivers/$driver" ] || {
    printf 'Error: declared driver directory is missing: drivers/%s\n' "$driver" >&2
    exit 1
  }
  destination="$SOURCE_DIR/libfprint/drivers/$driver"
  install -d "$destination"
  cp -a "$REPO_DIR/drivers/$driver/." "$destination/"
done

for module in "$REPO_DIR"/lib/goodix/*; do
  [ -d "$module" ] || continue
  destination="$SOURCE_DIR/libfprint/lib/goodix/$(basename "$module")"
  install -d "$destination"
  cp -a "$module/." "$destination/"
done

install -d \
  "$SOURCE_DIR/goodix-spi" \
  "$SOURCE_DIR/libfprint/drivers/goodix-spi" \
  "$SOURCE_DIR/tests/goodix-spi"
install -m 0644 "$REPO_DIR/integration/libfprint-project/meson.build" \
  "$SOURCE_DIR/goodix-spi/meson.build"
install -m 0644 "$REPO_DIR/integration/libfprint-drivers/meson.build" \
  "$SOURCE_DIR/libfprint/drivers/goodix-spi/meson.build"
install -m 0644 "$REPO_DIR/integration/libfprint-tests/meson.build" \
  "$SOURCE_DIR/tests/goodix-spi/meson.build"

for test_directory in "$REPO_DIR"/tests/goodix-*; do
  [ -d "$test_directory" ] || continue
  cp -a "$test_directory" "$SOURCE_DIR/tests/goodix-spi/"
done

if grep -Fq "subdir('goodix-spi')" "$SOURCE_DIR/meson.build" &&
   grep -Fq "subdir('drivers/goodix-spi')" "$SOURCE_DIR/libfprint/meson.build" &&
   grep -Fq "subdir('goodix-spi')" "$SOURCE_DIR/tests/meson.build"; then
  printf 'libfprint integration already appears to be applied.\n'
elif git -C "$SOURCE_DIR" apply --check "$PATCH_FILE"; then
  git -C "$SOURCE_DIR" apply "$PATCH_FILE"
  printf 'Applied %s\n' "$PATCH_FILE"
else
  printf 'Error: could not apply %s to %s.\n' "$PATCH_FILE" "$SOURCE_DIR" >&2
  exit 1
fi

# Cached trees prepared by an older integration revision already contain the
# three subdir() hooks, so the complete versioned patch cannot be re-applied.
# Add only the new project-owned option in that migration case. Fresh trees get
# the same text from the versioned patch above.
if ! grep -Fq "option('goodix_spi_developer'" "$SOURCE_DIR/meson_options.txt"; then
  printf '\n' >> "$SOURCE_DIR/meson_options.txt"
  cat "$OPTIONS_FRAGMENT" >> "$SOURCE_DIR/meson_options.txt"
  printf 'Added the Goodix SPI developer build option to the cached tree.\n'
fi
