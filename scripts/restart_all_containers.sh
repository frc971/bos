#!/bin/bash

HOST="$1"

if [ -z "$HOST" ]; then
  echo "Usage: $0 <host>"
  exit 1
fi


ssh -t "$HOST" '
containers=$(docker ps -q)
if [ -n "$containers" ]; then
    echo "Stopping running containers..."
    docker stop $containers
else
    echo "No running containers to stop."
fi
docker container prune -f
'

ssh -t "$HOST" "docker run -d --name main --restart unless-stopped --net host --privileged --runtime nvidia \
  -v /bos:/bos \
  -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
  -v /tmp/argus_socket:/tmp/argus_socket \
  -v /usr/lib/aarch64-linux-gnu/nvidia:/usr/lib/aarch64-linux-gnu/nvidia \
  -v /var/run:/var/run \
  -e LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/nvidia:/usr/lib/aarch64-linux-gnu/tegra:/bos/bin \
  ghcr.io/frc971/bos/orin /bin/bash -c \"cd /bos && build/main/localization_main \""
