#!/bin/bash

HOST="$1"

if [ -z "$HOST" ]; then
  echo "Usage: $0 <host>"
  exit 1
fi

rsync -avz "$HOST":/bos/logs .
