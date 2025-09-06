HOST="$1"

executables=("main/main" "main/calibration/frame_logger" "main/calibration/calibrate" "main/calibration/frame_logger")

for n in "${executables[@]}"; do
  cp $n bin
 done

mkdir -p bin
for n in `find -name "*.so"`; do
  cp $n bin
 done

 rsync -avz bin "$HOST":/bos/bin
