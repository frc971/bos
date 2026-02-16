#!/bin/bash

DIR="build/src/test/unit_test"

for file in "$DIR"/*; do
    if [[ -x "$file" && -f "$file" ]]; then
        echo "Running $file"
        "$file"
    fi
done

