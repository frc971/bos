#!/bin/bash

HOST="$1"

if [ -z "$HOST" ]; then
  echo "Usage: $0 <host>"
  exit 1
fi

rsync -avz --progress image.tar "$HOST":/bos/image.tar
ssh -t "$HOST" 'sudo docker load -i /bos/image.tar'
