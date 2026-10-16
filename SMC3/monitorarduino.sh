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
  echo "Usage: $0 [baudrate]" >&2
  exit 2
fi

baudrate=${1:-9600}
if [[ ! "$baudrate" =~ ^[0-9]+$ || "$baudrate" -eq 0 ]]; then
  echo "Error: baudrate must be a positive number." >&2
  exit 2
fi

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

echo "Monitoring Arduino Uno at $port ($fqbn) at ${baudrate} baud. Press Ctrl+C to exit."
exec arduino-cli monitor \
  --port "$port" \
  --fqbn "$fqbn" \
  --config "$baudrate"
