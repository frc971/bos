executables=("main/main" "main/calibration/frame_logger")

for n in $executables; do
  cp $n bin
 done

mkdir -p bin
for n in `find -type d -name "build" -prune -name "*.so"`; do
  cp $n bin
 done
