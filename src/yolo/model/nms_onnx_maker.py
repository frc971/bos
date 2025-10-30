from ultralytics import YOLO
import torch

model = YOLO("augmented.pt")
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model.export(format="onnx", device=device, nms=True)  # half=True uses FP16 for faster inference
