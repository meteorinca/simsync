#!/usr/bin/env bash

set -euo pipefail

if ! command -v arduino-cli >/dev/null 2>&1; then
  echo "Error: arduino-cli is not installed or is not in PATH." >&2
  exit 1
fi

if ! command -v jq >/dev/null 2>&1; then
  echo "Error: jq is required to detect the connected Arduino Uno." >&2
  exit 1
fi

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [sketch.ino]" >&2
  exit 2
fi

if [[ $# -eq 1 ]]; then
  sketch=$1
else
  mapfile -t sketches < <(find . -maxdepth 1 -type f -name '*.ino' -print | sort)
  if [[ ${#sketches[@]} -eq 0 ]]; then
    echo "Error: no .ino file found in the current directory." >&2
    exit 1
  elif [[ ${#sketches[@]} -gt 1 ]]; then
    echo "Error: more than one .ino file found. Choose one explicitly:" >&2
    printf '  %s\n' "${sketches[@]}" >&2
    exit 1
  fi
  sketch=${sketches[0]}
fi

if [[ ! -f "$sketch" ]]; then
  echo "Error: sketch not found: $sketch" >&2
  exit 1
fi

sketch=${sketch#./}
sketch_dir=$(dirname "$sketch")
sketch_name=$(basename "$sketch")

mapfile -t uno_boards < <(
  arduino-cli board list --format json 2>/dev/null |
    jq -r '
      .detected_ports[]?
      | . as $detected
      | $detected.matching_boards[]?
      | select(
          ((.fqbn // "") | ascii_downcase | contains("uno"))
          or
          ((.name // "") | ascii_downcase | contains("uno"))
        )
      | [$detected.port.address, (.fqbn // "")] | @tsv
    '
)

if [[ ${#uno_boards[@]} -eq 0 ]]; then
  echo "Error: no connected Arduino Uno was detected." >&2
  echo "Connect an Uno and check it with: arduino-cli board list" >&2
  exit 1
fi

IFS=$'\t' read -r port fqbn <<< "${uno_boards[0]}"
if [[ -z "$port" || -z "$fqbn" ]]; then
  echo "Error: the connected Uno did not report a usable port or FQBN." >&2
  exit 1
fi

echo "Uploading $sketch_name to Arduino Uno at $port ($fqbn)..."
arduino-cli compile --upload \
  --fqbn "$fqbn" \
  --port "$port" \
  "$sketch_dir"

echo "Upload complete."
