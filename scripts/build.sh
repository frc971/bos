#!/usr/bin/env bash
set -e

# Default name
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

git submodule init
git submodule update
cmake -DENABLE_CLANG_TIDY=OFF -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-O3" -B "$BUILD_DIR" -G Ninja .
cmake --build "$BUILD_DIR"
mkdir -p /bos
cp -r constants /bos