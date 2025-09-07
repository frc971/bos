#!/bin/bash

HOST="$1"

if [ -z "$HOST" ]; then
  echo "Usage: $0 <host>"
  exit 1
fi

# Copy public key to remote host
scp ~/.ssh/id_ed25519.pub "$HOST":/tmp/temp.pub

# Append key and clean up safely
ssh "$HOST" 'mkdir -p ~/.ssh && chmod 700 ~/.ssh && \
cat /tmp/temp.pub >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys && rm /tmp/temp.pub'

echo "Public key added to $HOST successfully."

