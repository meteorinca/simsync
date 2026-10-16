#!/usr/bin/env bash
# Build SimSync, including the embedded web UI, for Windows and/or Linux.
set -euo pipefail

TARGET="all"

usage() {
  cat << 'EOF'
Usage: ./build.sh [OPTIONS]

Options:
  --all        Build both Linux (simsync) and Windows (simsync.exe) [default]
  --linux      Build only Linux executable (simsync)
  --windows    Build only Windows executable (simsync.exe)
  -h, --help   Show this help message
EOF
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --all)
      TARGET="all"
      shift
      ;;
    --linux)
      TARGET="linux"
      shift
      ;;
    --windows)
      TARGET="windows"
      shift
      ;;
    -h|--help)
      usage
      ;;
    *)
      echo "Unknown option: $1" >&2
      echo "Use --help for usage details." >&2
      exit 2
      ;;
  esac
done

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

if ! command -v go >/dev/null 2>&1; then
  echo "Error: Go was not found in PATH. Install/activate Go, then run ./build.sh again." >&2
  exit 1
fi

build_target() {
  local os="$1"
  local arch="$2"
  local out="$3"
  local label="$4"
  local temp_file="./simsync-build.${os}.${arch}.$$.tmp"

  echo "Building ${label} (${os}/${arch}) -> ${out}..."
  if ! GOOS="${os}" GOARCH="${arch}" go build -trimpath -buildvcs=false -o "${temp_file}" ./cmd/simsync; then
    rm -f -- "${temp_file}"
    echo "Error: Failed to build ${label}." >&2
    exit 1
  fi

  if [[ "${os}" != "windows" ]]; then
    chmod +x "${temp_file}"
  fi

  mv -f -- "${temp_file}" "${out}"
  echo "Successfully built ${out}"
}

echo "=== SimSync Multi-Platform Build ==="

if [[ "$TARGET" == "all" || "$TARGET" == "windows" ]]; then
  build_target "windows" "amd64" "simsync.exe" "Windows Executable"
fi

if [[ "$TARGET" == "all" || "$TARGET" == "linux" ]]; then
  build_target "linux" "amd64" "simsync" "Linux Executable"
fi

echo ""
echo "Build complete."
if [[ "$TARGET" == "all" || "$TARGET" == "windows" ]]; then
  echo "  Windows: .\\simsync.exe --no-udp"
fi
if [[ "$TARGET" == "all" || "$TARGET" == "linux" ]]; then
  echo "  Linux:   ./simsync --no-udp"
fi
