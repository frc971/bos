#!/usr/bin/env bash
set -e

# Defaults
NAME=""

# Parse args
for arg in "$@"; do
  case $arg in
  --name=*)
    NAME="${arg#*=}"
    shift
    ;;
  esac
done

# Determine build directory
if [ -z "$NAME" ]; then
  BUILD_DIR="build"
else
  BUILD_DIR="${NAME}-build"
fi

if [ "$(pwd)" != "/bos" ]; then
  mkdir -p /bos/constants 2>/dev/null || sudo mkdir -p /bos/constants
  if [ -w /bos ]; then
    cp -r constants/. /bos/constants
  else
    sudo cp -r constants/. /bos/constants
  fi
fi
cmake -Wno-dev -DENABLE_CLANG_TIDY=OFF -DCMAKE_BUILD_TYPE=Release -B "$BUILD_DIR" -G Ninja .
cmake --build "$BUILD_DIR"
