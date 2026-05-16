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

### `scripts/download_constants.sh`

- Rsyncs `/bos/constants` from remote host into the current local directory.

Usage:

```bash
./scripts/download_constants.sh nvidia@10.99.71.11
```

### `scripts/download_logs.sh`

- Rsyncs `/bos/logs` from remote host into the current local directory.

Usage:

```bash
./scripts/download_logs.sh nvidia@10.99.71.11
```

## Log and Image Utilities

### `scripts/rsync_between.sh`

- Syncs only images whose filename timestamp seconds are in a given range.
- Works on a single remote folder.

Usage:

```bash
./scripts/rsync_between.sh nvidia@10.9.71.11:/bos/logs/log51/left ./local_logs 50 450
```

### `scripts/get_log.sh`

- Similar range-based sync, but runs for both `second_bot_left` and `second_bot_right` subfolders.

Usage:

```bash
./scripts/get_log.sh nvidia@10.9.71.11:/bos/logs/log51 ./local_logs 50 450
```

### `scripts/check_log.py`

- Interactive preview utility that grabs one representative image per chunk from remote folders.
- Uses OpenCV windows; press any key to continue, `Esc` to stop.
- Operates on `left` and `right` subfolders.

Usage:

```bash
python3 scripts/check_log.py nvidia@10.9.71.11:/bos/logs/log51 ./local_logs
```

### `scripts/check_log.sh`

- Python script (despite `.sh` extension) similar to `check_log.py`.
- Operates on `second_bot_left` and `second_bot_right` subfolders.

Usage:

```bash
python3 scripts/check_log.sh nvidia@10.9.71.11:/bos/logs/log51 ./local_logs
```

### `scripts/img_opener.py`

- Local timestamped image viewer built with Tkinter/Pillow.
- Shows images in timestamp order, optionally starting from a given time.

Usage:

```bash
python3 scripts/img_opener.py ./local_logs/left
python3 scripts/img_opener.py ./local_logs/left 120.0
```

### `scripts/log_culler.sh`

- Deletes images outside a timestamp range in a local folder.

Usage:

```bash
./scripts/log_culler.sh ./local_logs/left 50 450
```

### `scripts/delete-logs.sh`

- Removes `log1` through `logN` directories (with optional dry-run mode).

Usage:

```bash
./scripts/delete-logs.sh --last=30 --dir=/bos/logs --dry-run
./scripts/delete-logs.sh --last=30 --dir=/bos/logs
```

### `scripts/fix_log_times.sh`

- Python utility that normalizes WPILOG timestamps so the first non-zero timestamp becomes `0`.
- Writes `<input>_fixed.wpilog` unless output path is provided.

Usage:

```bash
python3 scripts/fix_log_times.sh input.wpilog
python3 scripts/fix_log_times.sh input.wpilog output.wpilog
```
