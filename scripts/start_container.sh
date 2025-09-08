#!/bin/bash

HOST="$1"

ssh -t "$HOST" "docker run --net host --rm --privileged --runtime nvidia -it \
  -v /bos:/bos \
  -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
  -v /tmp/argus_socket:/tmp/argus_socket \
  -v /usr/lib/aarch64-linux-gnu/nvidia:/usr/lib/aarch64-linux-gnu/nvidia \
  -v /var/run:/var/run \
  -e LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/nvidia:/usr/lib/aarch64-linux-gnu/tegra:/bos/bin \
  ghcr.io/frc971/bos/orin"

