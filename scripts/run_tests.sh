#!/bin/bash

DIR="build/src/test/unit_test"
FAILED=0

for file in "$DIR"/*; do
    if [[ -x "$file" && -f "$file" ]]; then
        echo "Running $file"
        "$file" || FAILED=1
    fi
done

exit $FAILED

