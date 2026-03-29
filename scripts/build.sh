#!/usr/bin/env bash
set -e

# Defaults
NAME=""
TYPE="Release"

# Parse args
for arg in "$@"; do
  case $arg in
    --name=*)
      NAME="${arg#*=}"
      shift
      ;;
    --type=*)
      TYPE="${arg#*=}"
      shift
      ;;
  esac
done

# Normalize / validate type
if [[ "$TYPE" != "Release" && "$TYPE" != "Debug" ]]; then
  echo "Invalid --type: $TYPE (must be Release or Debug)"
  exit 1
fi

# Lowercase version for directory naming
TYPE_LOWER=$(echo "$TYPE" | tr '[:upper:]' '[:lower:]')

# Determine build directory
if [ -z "$NAME" ]; then
  BUILD_DIR="${TYPE_LOWER}-build"
else
  BUILD_DIR="${NAME}-${TYPE_LOWER}-build"
fi

git submodule update --init --progress --depth 1
cmake -DENABLE_CLANG_TIDY=OFF -DCMAKE_BUILD_TYPE="$TYPE" -B "$BUILD_DIR" -G Ninja .
cmake --build "$BUILD_DIR"
mkdir -p /bos
cp -r constants /bos
