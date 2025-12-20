#!/bin/bash

cmake --build build

HOST="$1"

if [ -z "$HOST" ]; then
  echo "Usage: $0 <host>"
  exit 1
fi

./scripts/copy_to_bin.sh

rsync -avz bin "$HOST":/bos
rsync -avz constants "$HOST":/bos
