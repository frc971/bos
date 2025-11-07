#!/bin/bash

cmake --build build

CMD="$1"
HOST="$2"

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

echo "$CMD"
rsync -avz -e "$CMD" bin "$HOST":/bos/bin
rsync -avz -e "$CMD" constants "$HOST":/bos/constants
