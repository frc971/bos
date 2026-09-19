# Path camera simulator

This C++ tool samples every path in a PathPlanner auto, places the calibrated
camera at each robot pose, and saves rendered Open3D frames plus a JSON
manifest. Field and gamepiece poses use the standard WPILib blue-alliance
coordinate system: +X points away from the blue wall, +Y points left, and +Z
points up.

The simulator target currently uses PathPlanner's official 2026.1.2 Linux
ARM64 binary and therefore configures only on ARM64. It also requires the
Open3D C++ development package, OpenCV, and WPILib 2026. On Ubuntu 22.04,
Open3D is available as `libopen3d-dev`.

Build and run the included `Corner` example:

```sh
./scripts/build.sh
DISPLAY=:0 build/bin/path_camera_sim
```

Useful flags include `--auto_name`, `--pathplanner_dir`, `--camera`, `--fps`,
`--gamepieces`, `--output_dir`, and `--apply_distortion`. Run with `--help` for
their defaults. A gamepiece file has this form:

```json
{
  "gamepieces": [
    {
      "type": "Fuel",
      "translation_m": [4.35, 3.10, 0.075],
      "rotation_rpy_rad": [0.0, 0.0, 0.0]
    }
  ]
}
```

Generate a complete Fuel layout from the staged pieces in `field-cad/model.glb`
with the `generate_fuel_config` target:

```sh
build/bin/generate_fuel_config --entropy=0.4 --seed=7 \
  --output=sim-output/fuel_gamepieces.json
```

Entropy is a finite value from 0 to 1. At 0, all 456 Fuel poses exactly match
the unprocessed field model. As entropy increases, retained Fuel is displaced
horizontally with a standard deviation that grows to 2 m, and the removal
probability increases linearly to 50%. Displacement is clamped inside the
field. The seed makes layouts reproducible. Generated files record `entropy`
and `seed` as metadata and can be passed directly to
`path_camera_sim --gamepieces=...`.

The tool makes a cached copy of the field GLB under the output directory and
removes exactly the staged gamepiece mesh instances declared by the field
configuration. This prevents the field's baked-in Fuel from being rendered in
addition to the configured pieces. If Open3D cannot initialize the configured
display, the simulator retries once under `xvfb-run`; install Xvfb when running
without a desktop display. It reports an error if neither the configured
display nor Xvfb can provide an OpenGL context.
