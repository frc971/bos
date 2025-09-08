#!/bin/bash

HOST="$1"
CONTAINER="$2"

ssh -t "$HOST" "docker restart" "$CONTAINER"
