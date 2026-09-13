# Path camera simulator

This C++ tool samples every path in a PathPlanner auto, places the calibrated
camera at each robot pose, and saves rendered Open3D frames plus a JSON
manifest. Field and gamepiece poses use the standard WPILib blue-alliance
coordinate system: +X points away from the blue wall, +Y points left, and +Z
points up.

The simulator target currently uses PathPlanner's official 2026.1.2 Linux
ARM64 binary and therefore configures only on ARM64. It also requires the
Open3D C++ development package, OpenCV, WPILib 2026, and an accessible X11/
OpenGL display. On Ubuntu 22.04, Open3D is available as `libopen3d-dev`.

Build and run the included `Corner` example:

```sh
cmake -S . -B build-sim -DBUILD_PATH_CAMERA_SIM=ON -DENABLE_CLANG_TIDY=OFF
cmake --build build-sim --target path_camera_sim -j2
DISPLAY=:0 build-sim/bin/path_camera_sim
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

The tool makes a cached copy of the field GLB under the output directory and
removes exactly the staged gamepiece mesh instances declared by the field
configuration. This prevents the field's baked-in Fuel from being rendered in
addition to the configured pieces. Rendering requires a working display/OpenGL
environment; it intentionally does not substitute a synthetic or approximate
field if Open3D cannot initialize.
