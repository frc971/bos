# Scripts Reference

This document describes the scripts under `scripts/` and when to use them.

## Build and Static Analysis

### `scripts/build.sh`

- Main build entrypoint.
- Initializes submodules.
- Creates `/bos/constants` when running outside `/bos`.
- Configures a Release Ninja build with clang-tidy disabled.

Usage:

```bash
./scripts/build.sh
./scripts/build.sh --name=mybuild
```

### `scripts/clang_tidy_build.sh`

- Build helper with clang-tidy enabled.
- Uses `build/` as the output directory.
- Usually used for CI checks rather than day-to-day local builds.

Usage:

```bash
./scripts/clang_tidy_build.sh
```

## Test Execution

### `scripts/run_tests.sh`

- Runs all executable files in `build/src/test/unit_test`.
- Returns non-zero if any test fails.

Usage:

```bash
./scripts/run_tests.sh
```

## Deploy and Remote Sync

### `scripts/copy_to_bin.sh`

- Collects built executables from `build/src` into `bin/`.
- Also copies `.so` and `.a` artifacts from `build/` into `bin/`.

Usage:

```bash
./scripts/copy_to_bin.sh
```

### `scripts/deploy.sh`

- Builds project, stages binaries into `bin/`, then rsyncs `bin/` and `constants/` to remote `/bos`.
- Optional remote service restart (`bos.service`).

Usage:

```bash
./scripts/deploy.sh nvidia@10.99.71.11
./scripts/deploy.sh nvidia@10.99.71.11 true
```

### `scripts/remote_deploy.sh`

- Like `deploy.sh`, but lets you pass a custom remote shell command (for jump-host/tunnel setups).
- Current workflow uses a reverse tunnel through the build server.

Usage:

```bash
ssh -R 9000:localhost:22 root@build-server
./scripts/remote_deploy.sh "ssh -J charlie@localhost:9000" "nvidia@10.9.71.11"
```

### `scripts/upload_navgrid.sh`

- Copies a local navgrid file into the running `orin` Docker container.
- Then deploys to `nvidia@10.99.71.11` without restart.

Usage:

```bash
./scripts/upload_navgrid.sh
```

### `scripts/upload_public_key.sh`

- Uploads local `~/.ssh/id_ed25519.pub` and appends it to remote `authorized_keys`.

Usage:

```bash
./scripts/upload_public_key.sh nvidia@10.99.71.11
```
