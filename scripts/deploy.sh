#!/bin/bash

cmake --build build

HOST="$1"

if [ -z "$HOST" ]; then
  echo "Usage: $0 <host>"
  exit 1
fi

mkdir -p bin

binaries=(
  "build/main/localization_main"
  "build/main/test"
  "build/main/calibration/extrinsics_calibrate"
  "build/main/calibration/focus_calibrate"
  "build/main/calibration/frame_logger"
  "build/main/calibration/intrinsics_calibrate"
  "build/main/calibration/test_intrinsics"
)
for binary in "${binaries[@]}"; do
  if [[ -f "$binary" ]]; then
    cp "$binary" bin/
    echo "Copied: $binary"
  else
    echo "Warning: $binary not found."
  fi
done

find . -type f \( -name "*.so" -o -name "*.a" \) -exec cp {} bin/ \;

rsync -avz bin "$HOST":/bos
rsync -avz constants "$HOST":/bos
