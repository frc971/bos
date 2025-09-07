#!/bin/bash

HOST="$1"

if [ -z "$HOST" ]; then
  echo "Usage: $0 <host>"
  exit 1
fi

executables=("main/main" "main/calibration/frame_logger" "main/calibration/calibrate" "main/calibration/frame_logger" "main/camera/test" )

for n in "${executables[@]}"; do
  cp $n bin
 done

mkdir -p bin
for n in `find -name "*.so"`; do
  cp $n bin
 done

rsync -avz bin "$HOST":/bos
rsync -avz constants "$HOST":/bos
