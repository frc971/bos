from ultralytics import YOLO

# Load model
model = YOLO("/home/yasen/Training/runs/detect/train3/weights/last.pt")

# Run validation - it will automatically find your test set if you have a data.yaml
metrics = model.val(data="/mnt/c/Users/Yasen/Documents/grayscaled-gamepieces/data.yaml", split="test")

# Print metrics
print(f"mAP50: {metrics.box.map50}")
print(f"mAP50-95: {metrics.box.map}")
print(f"Precision: {metrics.box.p}")
print(f"Recall: {metrics.box.r}")
