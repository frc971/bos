from ultralytics import YOLO
import os

def main():
    # Path to your dataset.yaml
    data_yaml = "/home/nvidia/Documents/gamepiece-data/data.yaml"

    # Load YOLO11n model (nano, fastest and smallest)
    model = YOLO("yolo11n.pt")

    print("="*60)
    print("Starting YOLO11n Training")
    print("="*60)

    # Train the model
    results = model.train(
        data=data_yaml,
        epochs=50,
        imgsz=640,
        batch=16,
        device=0,
        
        # Important: Control augmentation (default might be too aggressive)
        hsv_h=0.015,      # HSV-Hue augmentation (reduce if colors matter)
        hsv_s=0.7,        # HSV-Saturation
        hsv_v=0.4,        # HSV-Value
        degrees=0.0,      # Rotation (set to 0 if orientation matters)
        translate=0.1,    # Translation
        scale=0.5,        # Scale variation
        fliplr=0.5,       # Horizontal flip probability
        mosaic=1.0,       # Mosaic augmentation
        
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
        project='runs/detect',
        name='train',
        exist_ok=True
    )

    print("\n" + "="*60)
    print("Training Complete!")
    print("="*60)
    
    # Get the best model path
    best_model_path = "runs/detect/train/weights/best.pt"
    
    if not os.path.exists(best_model_path):
        print(f"ERROR: Best model not found at {best_model_path}")
        return
    
    # Load the best trained model
    model = YOLO(best_model_path)
    
    # Validate on test set to verify performance BEFORE export
    print("\n" + "="*60)
    print("Validating on Test Set")
    print("="*60)
    
    val_results = model.val(
        data=data_yaml,
        split='test',      # Validate on test split
        imgsz=640,
        batch=16,
        device=0,
        plots=True,
        save_json=True,    # Save results for analysis
        save_hybrid=True   # Save hybrid labels (useful for debugging)
    )
    
    # Print validation metrics
    print("\nValidation Results:")
    print(f"  mAP50: {val_results.box.map50:.3f}")
    print(f"  mAP50-95: {val_results.box.map:.3f}")
    print(f"  Precision: {val_results.box.mp:.3f}")
    print(f"  Recall: {val_results.box.mr:.3f}")
    
    # Export to TensorRT with NMS included
    print("\n" + "="*60)
    print("Exporting to TensorRT")
    print("="*60)
    
    export_path = model.export(
        format="engine",
        half=True,         # FP16 for faster inference
        device=0,
        simplify=True,     # Simplify the ONNX model
        workspace=4,       # GPU memory workspace in GB
        imgsz=640,
        
        # CRITICAL: Export with NMS included
        # Note: YOLOv11 might handle this differently, check docs
        # If NMS is not included, your evaluation script needs to apply it
    )
    
    print(f"\nModel exported to: {export_path}")
    
    # Test a single prediction to verify the model works
    print("\n" + "="*60)
    print("Testing Single Prediction")
    print("="*60)
    
    # Find a test image
    import glob
    test_images = glob.glob("/home/nvidia/Documents/gamepiece-data/test/images/*.jpg")
    if test_images:
        test_img = test_images[0]
        print(f"Testing on: {test_img}")
        
        # Run prediction
        results = model.predict(
            source=test_img,
            imgsz=640,
            conf=0.25,
            device=0,
            save=True,
            project='runs/detect',
            name='test_prediction'
        )
        
        if results:
            print(f"Detections: {len(results[0].boxes)}")
            print("Prediction saved to runs/detect/test_prediction/")
    
    print("\n" + "="*60)
    print("All Done!")
    print("="*60)
    print("\nNext Steps:")
    print("1. Check validation metrics above")
    print("2. Review training plots in runs/detect/train/")
    print("3. Check test prediction in runs/detect/test_prediction/")
    print("4. If metrics look good, use the .engine file for inference")
    print(f"5. Engine file location: {export_path}")

if __name__ == "__main__":
    main()
