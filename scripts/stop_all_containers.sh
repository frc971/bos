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
