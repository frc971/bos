import onnxruntime as ort
import cv2
import numpy as np
from pathlib import Path
from typing import List, Tuple, Optional


class ONNXDetector:
    """ONNX-based object detection library."""
    
    def __init__(self, model_path: str):
        """
        Initialize the ONNX detector.
        
        Args:
            model_path: Path to the ONNX model file
        """
        self.model_path = model_path
        
        # Load ONNX session
        self.session = ort.InferenceSession(model_path)
        self.input_name = self.session.get_inputs()[0].name
        input_shape = self.session.get_inputs()[0].shape
        self.img_size = input_shape[2]
    
    def preprocess_image(self, img_rgb: np.ndarray) -> Tuple[np.ndarray, np.ndarray, dict]:
        """
        Preprocess image with letterbox resizing.
        
        Args:
            img_rgb: Input image in RGB format
            
        Returns:
            Tuple of (preprocessed image tensor, preprocessing metadata)
        """
        orig_h, orig_w = img_rgb.shape[:2]
        
        # Calculate scaling factor
        scale = min(self.img_size / orig_h, self.img_size / orig_w)
        new_h, new_w = int(orig_h * scale), int(orig_w * scale)
        
        # Resize image
        img_resized = cv2.resize(img_rgb, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        
        # Create padded image (letterbox)
        img_padded = np.full((self.img_size, self.img_size, 3), 114, dtype=np.uint8)
        
        # Calculate padding offsets
        top = (self.img_size - new_h) // 2
        left = (self.img_size - new_w) // 2
        
        # Place resized image in center
        img_padded[top:top+new_h, left:left+new_w] = img_resized
        
        # Normalize and transpose
        img_norm = img_padded.astype(np.float32) / 255.0
        img_input = np.transpose(img_norm, (2, 0, 1))[None, ...]
        
        # Store metadata for postprocessing
        metadata = {
            'scale': scale,
            'top': top,
            'left': left,
            'orig_h': orig_h,
            'orig_w': orig_w
        }
        
        return img_input, img_padded, metadata
    
    def detect(self, img_rgb: np.ndarray) -> List[Tuple[int, int, int, int, float, int]]:
        """
        Run detection on an image.
        
        Args:
            img_rgb: Input image in RGB format
            
        Returns:
            List of detections, each as (x1, y1, x2, y2, confidence, class_id)
        """
        # Preprocess
        img_input, img_padded, metadata = self.preprocess_image(img_rgb)
        
        # Inference
        outputs = self.session.run(None, {self.input_name: img_input})[0]
        
        # Postprocess
        detections = self._postprocess(outputs, metadata)
        
        return detections
    
    def _postprocess(self, outputs: np.ndarray, metadata: dict) -> List[Tuple[int, int, int, int, float, int]]:
        """
        Postprocess model outputs to get final detections.
        
        Args:
            outputs: Raw model outputs
            metadata: Preprocessing metadata
            
        Returns:
            List of detections in original image coordinates
        """
        detections = outputs[0]  # Remove batch dimension
        
        valid_detections = []
        scale = metadata['scale']
        top = metadata['top']
        left = metadata['left']
        
        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            
            # Convert coordinates back to original image space
            x1 = (x1 - left) / scale
            y1 = (y1 - top) / scale
            x2 = (x2 - left) / scale
            y2 = (y2 - top) / scale
            
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            valid_detections.append((x1, y1, x2, y2, float(conf), int(cls)))
        
        return valid_detections
    
    def draw_detections(self, img_bgr: np.ndarray, detections: List[Tuple], 
                       color: Tuple[int, int, int] = (0, 255, 0), 
                        thickness: int = 3):
        """
        Draw bounding boxes on image.
        
        Args:
            img_bgr: Input image in BGR format (for cv2)
            detections: List of detections from detect()
            color: BGR color tuple for boxes
            thickness: Line thickness for boxes
            
        Returns:
            Image with drawn detections
        """
        for x1, y1, x2, y2, conf, cls in detections:
            if (x1 == 0 and x2 == 0):
                continue;
            cv2.rectangle(img_bgr, (x1, y1), (x2, y2), color, thickness)
            
            label = f"{conf:.2f}"
            cv2.putText(img_bgr, label, (x1, max(y1-10, 0)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        


def resize_for_display(img: np.ndarray, max_size: int = 640) -> np.ndarray:
    """
    Resize image for display while maintaining aspect ratio.
    
    Args:
        img: Input image
        max_size: Maximum dimension size
        
    Returns:
        Resized image
    """
    h, w = img.shape[:2]
    scale = min(max_size / h, max_size / w)
    
    new_w = int(w * scale)
    new_h = int(h * scale)
    
    return cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)


def process_directory(img_dir: Path, model_path: str, 
                     display_size: int = 640):
    """
    Process all images in a directory and display detections.
    
    Args:
        img_dir: Directory containing images
        model_path: Path to ONNX model
        display_size: Display window size
    """
    # Find all image files
    img_files = (list(img_dir.glob('*.jpg')) + 
                 list(img_dir.glob('*.png')) + 
                 list(img_dir.glob('*.jpeg')))
    
    if len(img_files) == 0:
        print("ERROR: No images found!")
        return
    
    # Initialize detector
    detector = ONNXDetector(model_path)
    
    # Process each image
    for img_file in img_files:
        # Load image
        img_bgr = cv2.imread(str(img_file))
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        
        # Run detection
        detections = detector.detect(img_rgb)
        
        # Draw results
        detector.draw_detections(img_bgr, detections)
        
        # Resize for display
        img_display = resize_for_display(img_bgr, display_size)
        
        # Display
        cv2.imshow('Detections', img_display)
        key = cv2.waitKey(0)
        if key == ord('q'):
            break
    
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import constants
    
    img_dir = Path(constants.DATASET + "train/images")
    model_path = constants.MODEL_PATH
    
    process_directory(img_dir, model_path)
