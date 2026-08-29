#!/bin/sh

set -eu

PI_HOST="${PI_HOST:-[PUT YOUR PI HOST]}"
PI_LOG_DIR="${PI_LOG_DIR:-repo/flight-debrief/main/out}"

script_directory=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repository_root=$(CDPATH= cd -- "$script_directory/.." && pwd)
destination="${1:-$repository_root/downloaded-logs}"

if ! command -v rsync >/dev/null 2>&1; then
    echo "error: rsync is not installed" >&2
    exit 1
fi

mkdir -p "$destination"

echo "Downloading logs from $PI_HOST:~/$PI_LOG_DIR/"
echo "Destination: $destination"

rsync \
    --archive \
    --verbose \
    --partial \
    "$PI_HOST:~/$PI_LOG_DIR/" \
    "$destination/"

echo "Log download complete."
