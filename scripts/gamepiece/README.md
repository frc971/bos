## Making the Python Environment
- TODO use uv and write out list of files.
- NOTE: Separate envs are currently required for autolabeler and main training/testing
## Dataset Creation
### Getting Images
- Download dataset from Kaggle or Roboflow preferrably already labeled
- NOTE: If you concatenate multiple datasets, ensure that the names of the labels are the same (don't end up training on Coral vs CORAL)
  - Convention: Make everything lowercase
- Put everything into a central folder
- Fallbacks for shortage of labeled data:
  - Find a dataset of similarly shaped objects and use the `color_shifter.py` make it approximately the same
  - Make your own dataset: `video_clip_extractor.py` (NOT WORKING because ffmpeg and video scraping are hard)
  - (Currently not working) Run autolabeler on the images if the labels are sketch or missing
- Use `move_dataset.py` to move images between folders
- `dataset_image_extractor.py` to remove the distinction of train/valid/test if it doesn't matter
### Validating Dataset/Autodistill
- Use `draw_detections.py` and run through the images to make sure the detections look reasonable
- `img_printer.py` because I don't like windows
### Image Modifications (NEEDS TESTING)
- Most useful modifications are Shear, Rotate, Mosaic (always run mosaic), Translate, Scale, Blur, Noise
  - Noise isn't necessary for better cameras, should check what the actual feed looks like
  - Same for blur
- Don't go too overboard especially if the dataset is small, can cause overfitting
- Decision for grayscale: uniformly shaped objects could probably use grayscale, which will be much faster. Otherwise, color can help a fair amount. Alternative (UNTESTED): Use gray detection with manual pixel color verification
## Training
### GPU options
- Free: Google Collab or Kaggle
  - Google Collab free is insanely slow, also need to keep browser tab open
  - Idk about Kaggle
- If you have a PC with reasonable graphics card, use that. Ex: 4070 made a usable coral detector on 150 epochs on several thousand images in ~1 hour
  - Run `gpu_specs_inspector.py` to check
### Running the Train
- Run the `v2_train.py` script to start train
- Make sure to deposit to a unique position, save every n epochs to prevent random chance from deleting your progress
- Run `onnx_exporter.py` to convert pt binary to ONNX format with NMS ENABLED
  - It is extremely important that the binary be in NMS format, otherwise it is difficult to work with. Agents love to assume that NMS isn't enabled.
## Validation (NEEDS IMPROVEMENT)
- After receiving the pt binary, test it using `pt_tester.py`
  - Can also run `onnx_tester.py`, but I don't remember if this works
- On the device that is actually running the model (should be Jetson Orin):
  - Run: `/usr/src/tensorrt/bin/trtexec --onnx=<onnx_file_name>.onnx --saveEngine=<DESCRIPTIVE_engine_name>.engine --fp16`
  - Either `claude_validation.py` or `test.py` or `simple_test.py` or `old_model_tester.py` (all of them are sketch, needs testing)

  - Note: The validators often perform differently on onnx/pt (running using simple YOLO python functions) vs .engine (running in C++). TODO do more validation

