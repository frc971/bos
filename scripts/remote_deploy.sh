#!/bin/bash

cmake --build build

CMD="$1"
REMOTE="$2"

./scripts/copy_to_bin.sh

rsync -avz -e "$CMD" bin "$REMOTE":/bos
rsync -avz -e "$CMD" constants "$REMOTE":/bos
