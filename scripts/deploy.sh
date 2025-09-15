#!/bin/bash

make -j5

HOST="$1"

if [ -z "$HOST" ]; then
  echo "Usage: $0 <host>"
  exit 1
fi

executables=("main/main" "main/calibration/frame_logger" "main/calibration/intrinsics_calibrate" "main/calibration/extrinsics_calibrate" "main/calibration/frame_logger" "main/calibration/test_intrinsics" "main/calibration/focus_calibrate")

for n in "${executables[@]}"; do
  cp $n bin
 done

mkdir -p bin
for n in `find -name "*.so"`; do
  cp $n bin
 done

rsync -avz bin "$HOST":/bos
rsync -avz constants "$HOST":/bos
