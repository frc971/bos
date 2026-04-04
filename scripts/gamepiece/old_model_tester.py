import onnxruntime as ort
import cv2
import numpy as np
from pathlib import Path
import constants

# Config
img_dir = Path(constants.DATASET + "train/images")
MODEL_PATH = constants.MODEL_PATH

if not img_dir.is_dir():
    print("dataset doesn't exist")

img_files = list(img_dir.glob('*.jpg')) + list(img_dir.glob('*.png')) + list(img_dir.glob('*.jpeg'))

if len(img_files) == 0:
    print("ERROR: No images found!")
    exit(1)

session = ort.InferenceSession(MODEL_PATH)
input_name = session.get_inputs()[0].name
input_shape = session.get_inputs()[0].shape
img_size = input_shape[2]

CONF_THRESHOLD = 0.25
DISPLAY_SIZE = 640  # Display window size

for img_file in img_files:
    # Load image in RGB (OpenCV loads as BGR)
    img_bgr = cv2.imread(str(img_file))
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    
    # Letterbox resize (maintain aspect ratio with padding)
    orig_h, orig_w = img_rgb.shape[:2]
    
    # Calculate scaling factor
    scale = min(img_size / orig_h, img_size / orig_w)
    new_h, new_w = int(orig_h * scale), int(orig_w * scale)
    
    # Resize image
    img_resized = cv2.resize(img_rgb, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    
    # Create padded image (letterbox)
    img_padded = np.full((img_size, img_size, 3), 114, dtype=np.uint8)
    
    # Calculate padding offsets
    top = (img_size - new_h) // 2
    left = (img_size - new_w) // 2
    
    # Place resized image in center
    img_padded[top:top+new_h, left:left+new_w] = img_resized
    
    # Normalize and transpose
    img_norm = img_padded.astype(np.float32) / 255.0
    img_input = np.transpose(img_norm, (2, 0, 1))[None, ...]

    # Inference
    outputs = session.run(None, {input_name: img_input})[0]

    # Prepare image for drawing (use BGR for cv2.imshow)
    img_draw = img_bgr.copy()

    # Remove batch dimension
    detections = outputs[0]  # Now shape is (300, 6)

    num_valid = 0
    for det in detections:
        x1, y1, x2, y2, conf, cls = det

        # Filter by confidence threshold
        if conf > CONF_THRESHOLD:
            num_valid += 1
            
            # Convert coordinates back to original image space
            # Remove padding offset
            x1 = (x1 - left) / scale
            y1 = (y1 - top) / scale
            x2 = (x2 - left) / scale
            y2 = (y2 - top) / scale
            
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            cv2.rectangle(img_draw, (x1, y1), (x2, y2), (0, 255, 0), 6)

            label = f"{conf:.2f}"
            cv2.putText(img_draw, label, (x1, max(y1-10, 0)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Resize for display while maintaining aspect ratio
    display_h, display_w = img_draw.shape[:2]
    display_scale = min(DISPLAY_SIZE / display_h, DISPLAY_SIZE / display_w)
    
    new_display_w = int(display_w * display_scale)
    new_display_h = int(display_h * display_scale)
    
    img_display = cv2.resize(img_draw, (new_display_w, new_display_h), interpolation=cv2.INTER_LINEAR)
    
    cv2.imshow('Detections', img_display)
    key = cv2.waitKey(0)
    if key == ord('q'):
        break

cv2.destroyAllWindows()
