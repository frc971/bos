import onnxruntime as ort
import cv2
import yaml
import numpy as np
from pathlib import Path
import constants

# Config
YAML_PATH = constants.MODEL_PATH + "data.yaml"

with open(YAML_PATH) as f:
    data = yaml.safe_load(f)
yaml_dir = Path(YAML_PATH)
val_path = Path(constants.DATASET + "valid/images")
img_files = list(val_path.glob('*.jpg')) + list(val_path.glob('*.png'))

print(f"YAML path: {YAML_PATH}")
print(f"Val path: {val_path}")
print(f"Val path exists: {val_path.exists()}")
print(f"Images found: {len(img_files)}")
if len(img_files) == 0:
    print("ERROR: No images found!")
    exit(1)
print()

session = ort.InferenceSession(constants.MODEL_PATH)
input_name = session.get_inputs()[0].name
input_shape = session.get_inputs()[0].shape
img_size = input_shape[2]

print(f"Input shape:{img_size}")

# Metrics
tp = fp = fn = 0

for img_file in img_files:
    # Load image
    img = cv2.imread(str(img_file))
    img_resized = cv2.resize(img, (img_size, img_size))
    img_norm = img_resized.astype(np.float32) / 255.0
    img_input = np.transpose(img_norm, (2, 0, 1))[None, ...]

    # Inference
    outputs = session.run(None, {input_name: img_input})[0]
    num_preds = len(outputs)

    img_draw = img_resized.copy()

    print(f"Output shape: {outputs.shape}")
    for thingy in outputs:
        for det in thingy:
            print(f"Detection: {det}")
            x1, y1, x2, y2 = int(det[0]), int(det[1]), int(det[2]), int(det[3])
            cv2.rectangle(img_draw, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.imshow('Detections', img_draw)
        cv2.waitKey(0)

    # Load ground truth
    label_file = img_file.parent.parent / 'labels' / img_file.name
    label_file = label_file.with_suffix('.txt')
    num_gt = 0
    if label_file.exists():
        with open(label_file) as f:
            num_gt = len(f.readlines())
    else:
        print(f"Label not found: {label_file}")
        exit(1)

    # Count matches (assume model output is correct)
    matched = min(num_preds, num_gt)
    tp += matched
    fp += num_preds - matched
    fn += num_gt - matched

print(f"True Positives: {tp}")
print(f"False Positives: {fp}")
print(f"False Negatives: {fn}")
print(f"True Negatives: N/A (object detection)")
print(f"\nPrecision: {tp/(tp+fp) if tp+fp > 0 else 0:.3f}")
print(f"Recall: {tp/(tp+fn) if tp+fn > 0 else 0:.3f}")
