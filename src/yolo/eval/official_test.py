from ultralytics import YOLO

# Load TensorRT engine
model = YOLO("/bos/src/yolo/model/fifthYOLO.engine")

# Run inference
results = model("/home/nvidia/Documents/gamepiece-data/test/images/20250122_101406_jpg.rf.0eacf8c2b7e1e10ea6520ff58ccba153.jpg")

print("Results:", results)
