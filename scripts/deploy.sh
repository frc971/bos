#!/bin/bash

cmake --build build

HOST="$1"

if [ -z "$HOST" ]; then
  echo "Usage: $0 <host>"
  exit 1
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

rsync -avz --delete bin "$HOST":/bos
rsync -avz --delete constants "$HOST":/bos
ssh $HOST 'sudo systemctl restart bos.service'
