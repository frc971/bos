# Unit Tests

This directory contains GoogleTest-based checks for localization solvers, transformation math, pathfinding, and shared test helpers.

## Files

- `CMakeLists.txt` builds the unit test executables.
- `general_solver_test.cc` compares `SquareSolver` and `MultiTagSolver` on synthetic detections and a real AprilTag image.
- `joint_solve_test.cc` exercises `JointSolver` from a perturbed starting pose and compares it against square-solver output.
- `matrix.cc` validates transformation decomposition and recomposition helpers from `utils::transform`.
- `multi_tag_test.cc` checks that a single-tag multi-tag solve matches the square-solver pose matrix.
- `pathing_test.cc` loads the navigation grid and verifies BFS returns the expected node sequence.
- `square_solve_test.cc` validates basic `SquareSolver` orientation behavior from shared fake detections.
- `unit_test_utils.h` declares fake detections, numeric tolerances, pose print support, and comparison helpers for tests.
- `unit_test_utils.cc` implements GoogleTest pose printing and approximate `position_estimate_t` equality helpers.
- `test_image.jpg` is an image fixture available to unit tests.
