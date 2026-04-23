#!/bin/bash

cmake --build build

HOST="$1"

if [ -z "$HOST" ]; then
  echo "Usage: $0 <host> <restart>"
  exit 1
fi

RESTART="$2"
if [ "$RESTART" == "true" ]; then
  RESTART=true
else
  RESTART=false
fi

./scripts/copy_to_bin.sh

target=$(echo "$HOST" | sed 's/nvidia@//g')

echo "Waiting for remote $target"

while true; do
  # send a single ping; use default system behaviour
  if ping -c 1 "$target" >/dev/null 2>&1; then
    # success: exit 0 so && chains
    echo "Remote found!"
    break
  fi
  sleep 1
done

rsync -avh --delete bin "$HOST":/bos
rsync -avh --delete constants "$HOST":/bos
if [ "$RESTART" = true ]; then
  ssh "$HOST" 'sudo systemctl restart bos.service'
fi