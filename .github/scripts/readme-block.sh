#!/bin/sh
# Prints the code block that follows "<!-- test: NAME -->" in the README, so the
# install checks run the very commands the README gives, and cannot drift.
#     readme-block.sh NAME [README]
set -eu
block=$(awk -v name="$1" '
    $0 == "<!-- test: " name " -->" { armed = 1; next }
    armed && /^```/ { if (inside) exit; inside = 1; next }
    inside { print }
' "${2:-README.md}")
[ -n "$block" ] || { echo "no block named $1 in the README" >&2; exit 1; }
printf '%s\n' "$block"
