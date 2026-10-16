#!/usr/bin/env bash
# Build SimSync, including the embedded web UI, from any working directory.
set -euo pipefail

if [[ $# -gt 0 ]]; then
  if [[ $# -eq 1 && ( $1 == --help || $1 == -h ) ]]; then
    echo "Usage: ./build.sh"
    echo "Builds the simsync executable beside this script."
    exit 0
  fi
  echo "Usage: ./build.sh" >&2
  exit 2
fi

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

if ! command -v go >/dev/null 2>&1; then
  echo "Error: Go was not found in PATH. Install/activate Go, then run ./build.sh again." >&2
  exit 1
fi

# Replace the executable only after a successful build. A running instance
# continues using its old executable until the user stops and restarts it.
build_file=$(mktemp ./simsync-build.XXXXXX)
trap 'rm -f -- "$build_file"' EXIT

echo "Building SimSync and its web UI..."
if ! go build -trimpath -buildvcs=false -o "$build_file" ./cmd/simsync; then
  echo "Build failed. The previous simsync executable is still available." >&2
  exit 1
fi
chmod +x "$build_file"
mv -f -- "$build_file" ./simsync

echo "Build complete."
echo "Stop SimSync if it is already running, then start it with:"
echo "  ./simsync --no-udp"
echo "For game telemetry, use: ./simsync"
