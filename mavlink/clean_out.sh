#!/bin/sh

set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
out_dir="$script_dir/out"

if [ ! -d "$out_dir" ]; then
    printf 'Output directory does not exist: %s\n' "$out_dir"
    exit 1
fi

deleted=0

for file in "$out_dir"/*.csv; do
    [ -e "$file" ] || continue

    printf 'Deleting: %s\n' "$file"
    rm -- "$file"
    deleted=$((deleted + 1))
done

printf 'Deleted %d CSV file(s).\n' "$deleted"
