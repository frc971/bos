#!/bin/bash

cmake --build build

CMD="$1"
REMOTE="$2"

mkdir -p bin

binaries=(
  "build/src/localization_main"
  "build/src/test"
  "build/src/calibration/extrinsics_calibrate"
  "build/src/calibration/focus_calibrate"
  "build/src/calibration/frame_shower"
  "build/src/calibration/intrinsics_calibrate"
  "build/src/calibration/test_intrinsics"
  "build/src/calibration/frame_logger"
  "build/src/test/apriltag_detect_test"
  "build/src/test/tag_estimator_test"
  "build/src/test/intrinsics_test"
  "build/src/test/camera_source_test"
  "build/src/test/timer_test"
  "build/src/test/yolo_test"
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

rsync -avz -e "$CMD" bin "$REMOTE":/bos
rsync -avz -e "$CMD" constants "$REMOTE":/bos
