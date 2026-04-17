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
  mkdir -p /bos
  cp -r constants /bos
fi
git submodule update --init --progress --depth 1
ARCH=$(uname -m)
if [[ "$ARCH" == "x86_64" ]]; then
	echo asdf
	cmake -DENABLE_CLANG_TIDY=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake -B "$BUILD_DIR" -G Ninja .
else
	cmake -DENABLE_CLANG_TIDY=OFF -DCMAKE_BUILD_TYPE=Release -B "$BUILD_DIR" -G Ninja .
fi
cmake --build "$BUILD_DIR"
