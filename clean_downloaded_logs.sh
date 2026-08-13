#!/bin/sh

set -eu

script_directory=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
target="$script_directory/downloaded-logs"

if [ ! -d "$target" ]; then
    mkdir -p "$target"
    echo "Created empty directory: $target"
    exit 0
fi

if [ "${1:-}" != "--force" ]; then
    printf 'Delete everything inside %s? [y/N] ' "$target"
    read -r answer

    case "$answer" in
        y|Y|yes|YES) ;;
        *) echo "Cancelled."; exit 0 ;;
    esac
fi

find "$target" -mindepth 1 -maxdepth 1 -exec rm -rf -- {} +
echo "Cleaned: $target"
