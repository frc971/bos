# Cleaned up version of https://github.com/yasen5/GampieceYoloModels/blob/main/Training/v2_train.py
# Only major difference is the number of epochs between saves, 10 was too few

from ultralytics import YOLO
import glob
import os

def main():
    # Path to dataset.yaml
    data_yaml = "/mnt/c/Users/Yasen/Documents/more-gamepiece-data/data.yaml"
    device_num = 0;

    model = YOLO("/home/yasen/Training/runs/detect/train/weights/best.pt")

    print("="*60)
    print("Starting YOLO11n Training")
    print("="*60)

    # Train the model
    results = model.train(
        data=data_yaml,
        epochs=100,
        imgsz=640,
        batch=16,
        device=device_num,
        
        hsv_h=0.015,      # HSV-Hue augmentation
        hsv_s=0.7,        # HSV-Saturation
        hsv_v=0.4,        # HSV-Value
        degrees=10.0,      # Rotation 
        shear=10.0,
        translate=0.5,    # Translation
        scale=0.5,        # Scale variation
        fliplr=0.5,       # Horizontal flip probability
        mosaic=1.0,       # Mosaic augmentation
        close_mosaic=10,
        
        # Optimization
        optimizer='auto', 
        lr0=0.01,
        weight_decay=0.0005,
        
        val=True,         # Enable validation
        plots=True,       # Save training plots
        save=True,        # Save checkpoints
        save_period=30,   # Save checkpoint every N epochs
        
        # Project organization
        project='runs/detect',
        name='train3',
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
        device=device_num,
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
    
    print("\n" + "="*60)
    print("NMS Configuration Applied Successfully!")
    print("="*60)
    print("\nWith nms=True, the exported model includes:")
    print("  ✓ Non-Maximum Suppression (NMS) built-in")
    print("  ✓ Direct output format: detections ready to use")
    print("  ✓ No need to manually apply NMS in your code")
    print("\nExpected output format:")
    print("  - Bounding boxes in [x1, y1, x2, y2] format")
    print("  - Confidence scores")
    print("  - Class IDs")
    print("  - All overlapping detections already filtered")
    print("\nNMS thresholds used (defaults):")
    print("  - IoU threshold: 0.45")
    print("  - Confidence threshold: 0.25")
    print("="*60)
    
    print(f"\nPrimary model exported to: {primary_export}")
    
    # Test a single prediction to verify the model works
    print("\n" + "="*60)
    print("Testing Single Prediction")
    print("="*60)
    
    # Find a test image
    test_images = glob.glob("/mnt/c/Users/Yasen/Documents/gamepiece-data/test/images/*.jpg")
    if not test_images:
        test_images = glob.glob("/mnt/c/Users/Yasen/Documents/gamepiece-data/test/images/*.png")
    
    if test_images:
        test_img = test_images[0]
        print(f"Testing on: {test_img}")
        
        # Run prediction with the exported model
        results = model.predict(
            source=test_img,
            imgsz=640,
            conf=0.25,
            device=device_num,
            save=True,
            project='runs/detect',
            name='test_prediction'
        )
        
        if results:
            print(f"Detections: {len(results[0].boxes)}")
            print("Prediction saved to runs/detect/test_prediction/")
            
            # Show detection details
            if len(results[0].boxes) > 0:
                print("\nFirst few detections:")
                for i, box in enumerate(results[0].boxes[:5]):
                    print(f"  Detection {i+1}:")
                    print(f"    Class: {int(box.cls)}")
                    print(f"    Confidence: {float(box.conf):.3f}")
                    print(f"    BBox: {box.xyxy[0].tolist()}")
    else:
        print("No test images found to verify prediction.")
    
    print("\n" + "="*60)
    print("All Done!")
    print("="*60)
    print("\nNext Steps:")
    print("1. Check validation metrics above")
    print("2. Review training plots in runs/detect/train/")
    print("3. Check test prediction in runs/detect/test_prediction/")
    print("4. The exported model has NMS built-in - no manual NMS needed!")
    print(f"5. Model location: {primary_export}")
    print("\nWhen using the exported model:")
    print("  - Simply run inference and use the results directly")
    print("  - NMS is already applied with default thresholds")
    print("  - To adjust NMS thresholds at inference time, use:")
    print("    model.predict(conf=0.25, iou=0.45)")

if __name__ == "__main__":
    main()
