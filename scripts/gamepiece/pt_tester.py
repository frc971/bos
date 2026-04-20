from pathlib import Path
from ultralytics import YOLO
import cv2

# Config
img_dir = Path("datasets/rebuilt_balls/valid/images")
MODEL_PATH = "/home/yasen/runs/detect/rebuilt/weights/best.pt"

img_files = list(img_dir.glob('*.jpg')) + list(img_dir.glob('*.png'))

if len(img_files) == 0:
    print("ERROR: No images found!")
    exit(1)

# Load model
model = YOLO(MODEL_PATH)

CONF_THRESHOLD = 0.25

for img_file in img_files:
    print(f"\nProcessing: {img_file.name}")
    
    # Run inference
    results = model.predict(
        source=str(img_file),
        conf=CONF_THRESHOLD,
        save=False,
        verbose=True  # Print detection info
    )
    
    # Get annotated image
    annotated_img = results[0].plot()
    
    # Display
    cv2.imshow('Detections', annotated_img)
    key = cv2.waitKey(0)
    if key == ord('q'):
        break

cv2.destroyAllWindows()
