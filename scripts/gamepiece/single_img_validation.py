import onnxruntime as ort
import cv2
import numpy as np
from pathlib import Path

# Config
MODEL_PATH = "best.onnx"
IMAGE_PATH = "YellowBall.jpg"  # Change this to your image path

# Load model
session = ort.InferenceSession(MODEL_PATH)
input_name = session.get_inputs()[0].name
input_shape = session.get_inputs()[0].shape
img_size = input_shape[2]

print(f"Input shape: {img_size}")

# Load image
img_file = Path(IMAGE_PATH)
if not img_file.exists():
    print(f"ERROR: Image not found at {IMAGE_PATH}")
    exit(1)

img = cv2.imread(str(img_file))
img_resized = cv2.resize(img, (img_size, img_size))
img_norm = img_resized.astype(np.float32) / 255.0
img_input = np.transpose(img_norm, (2, 0, 1))[None, ...]

# Inference
outputs = session.run(None, {input_name: img_input})[0]
num_preds = len(outputs)

img_draw = img_resized.copy()

print(f"Output shape: {outputs.shape}")
print(f"Number of predictions: {num_preds}")
print()

for thingy in outputs:
    for det in thingy:
        print(f"Detection: {det}")
        x1, y1, x2, y2 = int(det[0]), int(det[1]), int(det[2]), int(det[3])
        cv2.rectangle(img_draw, (x1, y1), (x2, y2), (0, 255, 0), 2)

# Display result
cv2.imshow('Detections', img_draw)
print("\nPress any key to close the window...")
cv2.waitKey(0)
cv2.destroyAllWindows()

# Load ground truth
label_file = img_file.parent.parent / 'labels' / img_file.name
label_file = label_file.with_suffix('.txt')
num_gt = 0
if label_file.exists():
    with open(label_file) as f:
        num_gt = len(f.readlines())
    print(f"\nGround truth objects: {num_gt}")
    print(f"Predicted objects: {num_preds}")
else:
    print(f"\nWarning: Label file not found at {label_file}")
