#!/bin/bash

cmake --build build

HOST="$1"

if [ -z "$HOST" ]; then
  echo "Usage: $0 <host>"
  exit 1
fi

rsync -avz build "$HOST":/bos
rsync -avz constants "$HOST":/bos
