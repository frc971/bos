#!/bin/bash

cmake --build build

HOST="$1"

if [ -z "$HOST" ]; then
  echo "Usage: $0 <host>"
  exit 1
fi

mkdir -p bin

binaries=(
  "build/src/localization_main"
  "build/src/test"
  "build/src/calibration/extrinsics_calibrate"
  "build/src/calibration/focus_calibrate"
  "build/src/calibration/frame_logger"
  "build/src/calibration/intrinsics_calibrate"
  "build/src/calibration/test_intrinsics"
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
