#!/bin/bash

usage() {
  echo "Usage: $0 --last=<number> [--dir=<path>] [--dry-run]"
  echo ""
  echo "  --last=N     Delete log1 through logN"
  echo "  --dir=PATH   Directory containing logs (default: current directory)"
  echo "  --dry-run    Print commands without executing them"
  exit 1
}

LAST=""
DIR="."
DRY_RUN=false

for arg in "$@"; do
  case $arg in
    --last=*)   LAST="${arg#*=}" ;;
    --dir=*)    DIR="${arg#*=}" ;;
    --dry-run)  DRY_RUN=true ;;
    *)          echo "Unknown argument: $arg"; usage ;;
  esac
done

if [[ -z "$LAST" ]]; then
  echo "Error: --last is required."
  usage
fi

if ! [[ "$LAST" =~ ^[0-9]+$ ]]; then
  echo "Error: --last must be a positive integer."
  exit 1
fi

for i in $(seq 1 "$LAST"); do
  TARGET="$DIR/log$i"
  if $DRY_RUN; then
    echo "[dry-run] sudo rm -rf $TARGET"
  else
    echo "Removing $TARGET..."
    sudo rm -rf "$TARGET"
  fi
done

echo "Done."
