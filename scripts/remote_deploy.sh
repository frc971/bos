#!/bin/bash

cmake --build build

CMD="$1"
REMOTE="$2"

rsync -avh --delete -e "$CMD" build/bin "$REMOTE":/bos
rsync -avh --delete -e "$CMD" build/lib "$REMOTE":/bos
rsync -avh --delete -e "$CMD" constants "$REMOTE":/bos

# rsync -avh --delete -e "ssh -J charlie@localhost:9000" constants nvidia@100.65.180.61:/bos
# ssh -R 9000:localhost:22 -p 9122 root@vpn-tommy.ihealth-eng.com
