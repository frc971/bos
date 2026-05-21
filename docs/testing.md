# Testing Guide

## Test Layout

Tests live under `src/test`:

- `src/test/unit_test`: GoogleTest-based unit tests.
- `src/test/integration_test`: integration executables for subsystems and end-to-end checks.

## Build Tests

Tests are part of the normal project build. Use the project build script:

```bash
./scripts/build.sh
```

If you use a named build directory, keep that same directory when running tests:

```bash
./scripts/build.sh --name=mybuild
```

## Run Tests

Use the test runner script instead of running test binaries manually:

```bash
./scripts/run_tests.sh
```

This script runs executable tests in `build/src/test/unit_test` and returns a failing exit code if any test fails.
