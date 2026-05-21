# Build and Run

## Prerequisites

You can get set up in two ways:

1. Get access to the build server over Tailscale.
2. Build your own BOS Docker container from `https://github.com/frc971/bos-docker`.

Also ensure CUDA binaries are on your shell `PATH` by adding this to your `~/.zshrc` or `~/.bashrc`:

```bash
export PATH="/usr/local/cuda-12.6/bin:$PATH"
```

If you are using the build server or the BOS Docker container, required toolchains and dependencies are already provided.

## Build

Use the project build script instead of running CMake manually:

```bash
./scripts/build.sh
```

Optional named build directory:

```bash
./scripts/build.sh --name=mybuild
```

## Main Runtime Binaries

After building, these are the primary app entry points:

- `main_bot_main`
- `second_bot_main`
- `unambiguous_first`
- `unambiguous_second`

## Calibration Tools

Built in `src/calibration`:

- `intrinsics_calibrate`
- `frame_shower`
- `focus_calibrate`
