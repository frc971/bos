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

After building, these are the primary app entry points (some deprecated):

- `main_bot_main`
- `second_bot_main`
- `unambiguous_first`
- `unambiguous_second`

## Calibration Tools

Built in `src/calibration`:

- `intrinsics_calibrate`
- `frame_shower`
- `focus_calibrate`

## Frame Analysis Tools

`localization_stretch` scans one folder of frames with the 971 GPU AprilTag
detector. Frames are ordered by a numeric filename stem when available (for
example, `12.300000.jpg`). A stretch may contain up to `max_gap` consecutive
frames without a tag; it starts and ends on frames where a tag was detected.

```bash
./build/bin/localization_stretch \
  --image_folder=/bos/frames/gamepiece_camera \
  --intrinsics=constants/gamepiece/intrinsics.json \
  --max_gap=3
```

The result includes the start and end files, total span, frames with tags,
internal gap frames, and elapsed time when filenames are numeric timestamps.
