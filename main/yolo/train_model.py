
# train_and_export_yolo11n.py

from ultralytics import YOLO

def main():
    # Path to your dataset.yaml (Roboflow export provides this inside the dataset folder)
    data_yaml = "/home/nvidia/Documents/gamepiece-data/data.yaml"

    # Load YOLO11n model (nano, fastest and smallest)
    model = YOLO("yolo11n.pt")

    # Train the model
    model.train(
        data=data_yaml,
        epochs=50,         # adjust based on your dataset size
        imgsz=640,         # input image size
        batch=16,          # adjust based on GPU memory
        device=0           # set to 'cpu' if no GPU available
    )

    # Export to TensorRT (.engine)
    # By default, the latest trained weights are saved in runs/detect/train/weights/best.pt
    model = YOLO("runs/detect/train/weights/best.pt")
    model.export(format="engine", half=True, device=0)  # half=True uses FP16 for faster inference

if __name__ == "__main__":
    main()
