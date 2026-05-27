# YOLO

This directory wraps TensorRT YOLO inference and stores model metadata used by gamepiece detection and tests.

## Files

- `CMakeLists.txt` builds the `yolo` library.
- `model_constants.h` defines `yolo::ModelInfo`, the available model enum, model engine paths, class names, and color/gray model settings.
- `yolo.h` declares the `yolo::Yolo` TensorRT wrapper, including inference, post-processing, object-angle calculation, and detection drawing APIs.
- `yolo.cc` implements TensorRT engine loading, CUDA/OpenCV preprocessing, inference execution, YOLO post-processing, detection drawing, and GPU resource cleanup.

## Main Types

- `yolo::ModelInfo` describes one serialized TensorRT model and its classes.
- `yolo::Yolo` owns the TensorRT runtime, engine, execution context, CUDA stream, and input/output buffers needed to run inference.
