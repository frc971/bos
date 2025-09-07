HOST="$1"
COMMAND="$2"

ssh -t "$HOST" "sudo docker run --net host --privileged \
  -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra \
  -v /bos:/bos \
  --rm \
  -e LD_LIBRARY_PATH=/bos/bin \
  ghcr.io/frc971/bos/orin /bin/bash -c 'cd /bos && $COMMAND'"

