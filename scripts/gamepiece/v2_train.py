from ultralytics import YOLO
import glob
import os

def main():
    # Path to your dataset.yaml
    data_yaml = "/home/yasen/Training/datasets/rebuilt_balls/data.yaml"
    device_num = 0;

    # Load YOLO11n model (nano, fastest and smallest)
    model = YOLO("/home/yasen/runs/detect/even_larger_yellow/weights/best.pt")

    print("="*60)
    print("Starting YOLO11n Training")
    print("="*60)

    # Train the model
    results = model.train(
        data=data_yaml,
        epochs=40,
        imgsz=640,
        batch=16,
        device=device_num,
        
        # Important: Control augmentation (default might be too aggressive)
        degrees=10.0,      # Rotation (set to 0 if orientation matters)
        shear=10.0,
        translate=0.5,    # Translation
        scale=0.5,        # Scale variation
        fliplr=0.5,       # Horizontal flip probability
        mosaic=1.0,       # Mosaic augmentation
        close_mosaic=10,
        
        # Optimization
        optimizer='auto',  # or 'SGD', 'Adam', 'AdamW'
        lr0=0.01,         # Initial learning rate
        weight_decay=0.0005,
        
        # Validation during training
        val=True,         # Enable validation
        plots=True,       # Save training plots
        save=True,        # Save checkpoints
        save_period=10,   # Save checkpoint every N epochs
        
        # Project organization
        name='rebuilt',
    )

    print("\n" + "="*60)
    print("Training Complete!")
    print("="*60)
    
if __name__ == "__main__":
    main()
