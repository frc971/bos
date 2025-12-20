#!/bin/bash

mkdir -p bin

binaries=( $(find build/src -type f -executable -print) )

for binary in "${binaries[@]}"; do
  if [[ -f "$binary" ]]; then
    cp "$binary" bin/
  fi
done

find . -type f \( -name "*.so" -o -name "*.a" \) -exec cp {} bin/ \;

