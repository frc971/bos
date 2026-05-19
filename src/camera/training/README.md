# Training Task: Write Your Own TrainingCamera

## Purpose

Implement `TrainingCamera` from scratch against the `camera::ICamera` interface from the information below, then prove it works by piping it through the GPU AprilTag detector on a real Orin.

This is a learn-by-doing task. The interface is given. The implementation is yours.

> ## NO AI USE
>
> **You may not use AI tools on this task.** No Copilot, no Cursor autocomplete, no ChatGPT, no Claude, no "just to explain it to me." If you cannot write a line yourself, ask a mentor.
>
> The only way to learn is to fail. AI removes the failure and removes the learning. If a mentor catches AI-generated code in your submission you start over.
>
> Disable AI autocomplete in your editor (Copilot, Cursor AI, Continue, JetBrains AI, etc.) so you do not violate this by accident.

## Prerequisites

- Docker container set up on your computer.
- `bos` repo cloned and building locally.
- You have read [`./camera.h`](./camera.h) and can describe what each method is supposed to do.

## Time box

4-8 hours of real work, spread over as many sessions as you need. If you are stuck for more than 30 minutes on the same problem, ask a mentor.

## Learning goals

- Implement a C++ interface from a header.
- Work with OpenCV `cv::Mat` and basic filesystem iteration.
- Wire your own class into an existing test harness.
- Cross-compile / deploy and run on the Orin.

## The interface

Your camera lives behind [`./camera.h`](./camera.h) — a local copy of `src/camera/camera.h` placed here so you do not have to read around the rest of the camera code. Open it.

You are implementing `camera::ICamera`. The contracts:

- `GetFrame() -> timestamped_frame_t`
  Return the next frame from your input source, in deterministic order, with a `timestamp` and the loaded image. If something goes wrong with a single frame, mark it `invalid = true` rather than crashing the pipeline. Once you have walked off the end of the input, behavior is up to you — but `IsDone()` must agree with whatever you decide.
  `timestamp` is a `double` in seconds, monotonically increasing across calls. For this task it is fine to fake it as `frame_index / 30.0` or similar.
- `Restart() -> void`
  Reset internal state so the next `GetFrame()` call returns the first image again. After `Restart()`, `IsDone()` must be false.
- `GetCameraConstant() const -> camera_constant_t`
  Return the camera constant this camera is impersonating. Look at `src/camera/camera_constants.h` for what `camera_constant_t` is and how to pick one.
- `IsDone() -> bool`
  True once there are no more frames to return. The default returns false — you must override.

That is the whole interface. Pick your own filenames, your own member variables, your own ordering rule. Document the ordering rule in a comment at the top of your file so a mentor reading it knows what to expect.

## The task

1. **Set up.** Confirm your Docker container builds the repo clean. Create `src/camera/training/training_camera.cc` and a matching `training_camera.h`. Wire them into `src/camera/training/CMakeLists.txt`.
2. **Implement `TrainingCamera`.** Read a directory of PNGs in order and serve them as timestamped frames. Implement `Restart` and `IsDone` honestly.
3. **Write your own test.** Model it on `src/test/integration_test/apriltag_detect_test.cc`. Construct your `TrainingCamera`, feed it into the GPU AprilTag detector, and print or log whatever tells you the pipeline is working. Put it under `src/camera/training/` and wire it into the CMakeLists there. **Do not copy the existing test file verbatim** — read it, understand it, then write yours.
4. **Run on an Orin.** Deploy your test binary to an actual Orin and run it against a real sample PNG directory. Watch the output. Convince yourself the detections make sense.

Ask a mentor for a sample PNG directory before step 4 — possibly earlier if you want one to develop against.

## Allowed references

- `src/camera/training/camera.h` (this is the interface)
- `src/camera/camera_constants.h`
- `src/test/integration_test/apriltag_detect_test.cc` — as a **model** for the shape of your own test, not to copy
- OpenCV docs (cv::Mat, cv::imread, etc.)
- C++ standard library docs (filesystem, chrono, etc.)
- Your mentors

## When you get stuck

You will get stuck. That is the task working as intended.

- Re-read the interface and your own code out loud.
- Write a smaller failing test that isolates the part you do not understand.
- Print things. Lots of things.
- Then ask a mentor — with a specific question, not "it does not work."

## Checkoff

A mentor will sit with you at an Orin and verify:

- Your `training_camera.cc` compiles and you can explain every method you wrote.
- Running your test on the Orin against a real PNG directory produces AprilTag detections that match what we expect for that directory.
- `Restart()` actually restarts — run the test, restart, run again, get the same frames.
- `IsDone()` flips correctly at the end of the directory.
- No AI was used. The mentor will ask you to explain pieces of your code on the spot.
