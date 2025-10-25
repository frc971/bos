import os
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit  # initializes CUDA driver automatically
from glob import glob
from tqdm import tqdm
from scipy.optimize import linear_sum_assignment

ENGINE_PATH = "/bos/src/yolo/fifthYOLO.engine"
DATASET_ROOT = "/home/nvidia/Documents/gamepiece-data"
SPLIT = "test"
IMG_SIZE = 640

CONF_THRESH = 0.1
IOU_THRESH = 0.45
NMS_THRESH = 0.45

DATASET_DIR = os.path.join(DATASET_ROOT, SPLIT, "images")
LABEL_DIR = os.path.join(DATASET_ROOT, SPLIT, "labels")

# -------------------------------
# Helper functions
# -------------------------------

def letterbox(image, new_shape=640, color=(114, 114, 114)):
    """Resize image to fit model input (same as YOLOv5 preprocessing)."""
    height, width = image.shape[:2]
    scale_ratio = min(new_shape / height, new_shape / width)
    new_width = int(round(width * scale_ratio))
    new_height = int(round(height * scale_ratio))
    
    pad_width = new_shape - new_width
    pad_height = new_shape - new_height
    pad_left = pad_width / 2
    pad_top = pad_height / 2

    resized_image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
    top = int(round(pad_top - 0.1))
    bottom = int(round(pad_top + 0.1))
    left = int(round(pad_left - 0.1))
    right = int(round(pad_left + 0.1))
    
    padded_image = cv2.copyMakeBorder(resized_image, top, bottom, left, right, 
                                      cv2.BORDER_CONSTANT, value=color)
    return padded_image, scale_ratio, (pad_left, pad_top)


def compute_iou(box1, box2, class1=None, class2=None):
    """
    Compute IoU between two boxes [x1, y1, x2, y2].
    If class IDs are provided and don't match, returns 0.
    
    Args:
        box1: [x1, y1, x2, y2]
        box2: [x1, y1, x2, y2]
        class1: optional class ID for box1
        class2: optional class ID for box2
    
    Returns:
        IoU value (0 if classes don't match)
    """
    # If class IDs are provided and don't match, return 0
    if class1 is not None and class2 is not None and class1 != class2:
        return 0.0
    
    inter_x1 = max(box1[0], box2[0])
    inter_y1 = max(box1[1], box2[1])
    inter_x2 = min(box1[2], box2[2])
    inter_y2 = min(box1[3], box2[3])
    inter_area = max(inter_x2 - inter_x1, 0) * max(inter_y2 - inter_y1, 0)
    
    box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
    box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
    union_area = box1_area + box2_area - inter_area
    
    return inter_area / (union_area + 1e-6)


def non_max_suppression(detections, iou_threshold=0.45):
    """
    Apply Non-Maximum Suppression to detections.
    
    Args:
        detections: List of [x1, y1, x2, y2, conf, class_id]
        iou_threshold: IoU threshold for suppression
    
    Returns:
        List of detections after NMS
    """
    if len(detections) == 0:
        return []
    
    # Convert to numpy array for easier manipulation
    detections = np.array(detections)
    
    # Sort by confidence (descending)
    sorted_indices = np.argsort(detections[:, 4])[::-1]
    detections = detections[sorted_indices]
    
    keep_indices = []
    
    while len(detections) > 0:
        # Keep the detection with highest confidence
        keep_indices.append(0)
        
        if len(detections) == 1:
            break
        
        # Compute IoU with remaining detections
        current_box = detections[0, :4]
        current_class = int(detections[0, 5])
        
        suppress_indices = [0]  # Always suppress the current box
        
        for i in range(1, len(detections)):
            other_box = detections[i, :4]
            other_class = int(detections[i, 5])
            
            # Only suppress if same class and high IoU
            if current_class == other_class:
                iou_val = compute_iou(current_box, other_box)
                if iou_val > iou_threshold:
                    suppress_indices.append(i)
        
        # Keep detections that were not suppressed
        keep_mask = np.ones(len(detections), dtype=bool)
        keep_mask[suppress_indices] = False
        detections = detections[keep_mask]
    
    # Return kept detections as list
    return [list(detections[i]) for i in keep_indices if i < len(detections)]


# -------------------------------
# TensorRT Inference Wrapper
# -------------------------------

class TRTInfer:
    def __init__(self, engine_path):
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        
        if self.engine is None:
            file_exists = "exists" if os.path.exists(engine_path) else "doesn't exist"
            raise RuntimeError(f"Engine loaded from {engine_path} is None. File {file_exists}.")
        
        self.context = self.engine.create_execution_context()
        
        input_names = []
        output_names = []
        
        for i in range(self.engine.num_io_tensors):
            tensor_name = self.engine.get_tensor_name(i)
            tensor_mode = self.engine.get_tensor_mode(tensor_name)
            if tensor_mode == trt.TensorIOMode.INPUT:
                input_names.append(tensor_name)
            else:
                output_names.append(tensor_name)
        
        self.input_name = input_names[0]
        self.output_name = output_names[0]
        
        self.stream = cuda.Stream()
        
        self.input_shape = self.engine.get_tensor_shape(self.input_name)
        self.output_shape = self.engine.get_tensor_shape(self.output_name)
        
        self.input_nbytes = trt.volume(self.input_shape) * np.dtype(np.float32).itemsize
        self.output_nbytes = trt.volume(self.output_shape) * np.dtype(np.float32).itemsize
        
        self.d_input = cuda.mem_alloc(self.input_nbytes)
        self.d_output = cuda.mem_alloc(self.output_nbytes)

    def infer(self, image, orig_shape):
        """
        Run inference and return raw output with metadata for coordinate conversion.
        """
        orig_height, orig_width = orig_shape

        # Preprocess
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        preprocessed_img, scale_ratio, padding = letterbox(image, IMG_SIZE)
        preprocessed_img = preprocessed_img.transpose(2, 0, 1)
        preprocessed_img = np.expand_dims(preprocessed_img, 0)
        preprocessed_img = np.ascontiguousarray(preprocessed_img, dtype=np.float32) / 255.0
        
        cuda.memcpy_htod_async(self.d_input, preprocessed_img, self.stream)
        
        self.context.set_tensor_address(self.input_name, int(self.d_input))
        self.context.set_tensor_address(self.output_name, int(self.d_output))
        
        self.context.execute_async_v3(stream_handle=self.stream.handle)
        
        output = np.empty(self.output_shape, dtype=np.float32)
        cuda.memcpy_dtoh_async(output, self.d_output, self.stream)
        self.stream.synchronize()

        return output, scale_ratio, padding


def process_predictions(output, scale_ratio, padding, orig_shape, conf_thresh=0.25, apply_nms=True):
    """
    Process model output and convert to normalized coordinates.
    Assumes output format: [batch, num_detections, 6] or [num_detections, 6]
    where each detection is [x1, y1, x2, y2, conf, class_id]
    
    Args:
        output: Model output array
        scale_ratio: Scale ratio from letterbox
        padding: Padding (pad_left, pad_top) from letterbox
        orig_shape: Original image shape (height, width)
        conf_thresh: Confidence threshold
        apply_nms: Whether to apply NMS
    
    Returns:
        List of predictions in normalized coordinates
    """
    pad_left, pad_top = padding
    orig_height, orig_width = orig_shape

    # Handle different output shapes
    if len(output.shape) == 3:
        output = output[0]  # Remove batch dimension
    
    # If shape is [6, N], transpose to [N, 6]
    if output.shape[0] == 6 and output.shape[1] > 6:
        output = output.T
    
    if output.shape[0] == 0:
        return []

    predictions = []
    for detection in output:
        x1, y1, x2, y2, conf, class_id = detection
        
        if conf < conf_thresh:
            continue
        
        # Convert from letterbox coordinates to original image coordinates
        x1 = (x1 - pad_left) / scale_ratio
        y1 = (y1 - pad_top) / scale_ratio
        x2 = (x2 - pad_left) / scale_ratio
        y2 = (y2 - pad_top) / scale_ratio
        
        # Clip to image bounds
        x1 = max(0, min(x1, orig_width))
        y1 = max(0, min(y1, orig_height))
        x2 = max(0, min(x2, orig_width))
        y2 = max(0, min(y2, orig_height))
        
        # Convert to normalized coordinates [0, 1]
        x1_norm = x1 / orig_width
        y1_norm = y1 / orig_height
        x2_norm = x2 / orig_width
        y2_norm = y2 / orig_height

        print("x1_norm:", x1_norm,"y1_norm:", y1_norm,"x2_norm:", x2_norm,"y2_norm:", y2_norm)
        exit(0)
        
        predictions.append([x1_norm, y1_norm, x2_norm, y2_norm, conf, int(class_id)])
    
    # Apply NMS if requested
    if apply_nms and len(predictions) > 0:
        predictions = non_max_suppression(predictions, NMS_THRESH)
    
    return predictions


# -------------------------------
# Evaluation Loop
# -------------------------------

def evaluate():
    # Verify dataset structure
    if not os.path.exists(DATASET_ROOT):
        print(f"ERROR: Dataset root not found: {DATASET_ROOT}")
        exit(1)
    
    if not os.path.exists(DATASET_DIR):
        print(f"ERROR: Image directory not found: {DATASET_DIR}")
        print(f"Expected structure: {DATASET_ROOT}/{SPLIT}/images/")
        exit(1)
    
    if not os.path.exists(LABEL_DIR):
        print(f"ERROR: Label directory not found: {LABEL_DIR}")
        print(f"Expected structure: {DATASET_ROOT}/{SPLIT}/labels/")
        exit(1)
    
    inferer = TRTInfer(ENGINE_PATH)
    image_files = sorted(glob(os.path.join(DATASET_DIR, "*.jpg")) + 
                        glob(os.path.join(DATASET_DIR, "*.png")))
    
    if len(image_files) == 0:
        print("ERROR: No images found in dataset directory!")
        exit(1)

    print(f"Found {len(image_files)} images\n")

    all_predictions = []
    all_ground_truths = []
    total_raw_detections = 0
    total_filtered_detections = 0
    
    for img_path in tqdm(image_files, desc="Running inference"):
        label_path = os.path.join(LABEL_DIR, 
                                 os.path.basename(img_path).rsplit(".", 1)[0] + ".txt")
        if not os.path.exists(label_path):
            print(f"WARNING: Label not found: {label_path}")
            continue

        image = cv2.imread(img_path)
        if image is None:
            print(f"ERROR: Failed to read image: {img_path}")
            continue
        
        orig_shape = image.shape[:2]
        output, scale_ratio, padding = inferer.infer(image, orig_shape)
        
        # Count raw detections before filtering
        if len(output.shape) == 3:
            raw_count = output.shape[1]
        else:
            raw_count = output.shape[0] if output.shape[0] != 6 else output.shape[1]
        total_raw_detections += raw_count
        
        predictions = process_predictions(output, scale_ratio, padding, orig_shape, 
                                         CONF_THRESH, apply_nms=True)
        total_filtered_detections += len(predictions)
        
        all_predictions.append(predictions)

        # Load ground truth in YOLO format (normalized coordinates)
        ground_truths = []
        with open(label_path) as f:
            for line in f.readlines():
                parts = line.strip().split()
                if len(parts) == 5:
                    class_id, x_center, y_center, width, height = map(float, parts)
                    x1 = x_center - width / 2
                    y1 = y_center - height / 2
                    x2 = x_center + width / 2
                    y2 = y_center + height / 2
                    ground_truths.append([x1, y1, x2, y2, class_id])
        all_ground_truths.append(ground_truths)

    # Compute metrics
    num_true_positives = 0
    num_false_positives = 0
    num_false_negatives = 0
    num_true_negatives = 0
    total_ground_truth_objects = sum(len(gts) for gts in all_ground_truths)

    for predictions, ground_truths in zip(all_predictions, all_ground_truths):
        matched_gt_indices = set()
        matched_pred_indices = set()
        
        if len(predictions) == 0 and len(ground_truths) == 0:
            # True negative case
            num_true_negatives += 1
        elif len(predictions) == 0:
            # All ground truths are false negatives
            num_false_negatives += len(ground_truths)
        elif len(ground_truths) == 0:
            # All predictions are false positives
            num_false_positives += len(predictions)
        else:
            # Build IoU matrix
            iou_matrix = np.zeros((len(predictions), len(ground_truths)))
            for pred_idx, pred in enumerate(predictions):
                pred_box = pred[:4]
                pred_class = pred[5]
                for gt_idx, gt in enumerate(ground_truths):
                    gt_box = gt[:4]
                    gt_class = gt[4]
                    iou_matrix[pred_idx, gt_idx] = compute_iou(pred_box, gt_box, 
                                                               pred_class, gt_class)
            
            # Use Hungarian algorithm for optimal matching
            pred_indices, gt_indices = linear_sum_assignment(-iou_matrix)
            
            # Apply matches that exceed IoU threshold
            for pred_idx, gt_idx in zip(pred_indices, gt_indices):
                if iou_matrix[pred_idx, gt_idx] > IOU_THRESH:
                    num_true_positives += 1
                    matched_gt_indices.add(gt_idx)
                    matched_pred_indices.add(pred_idx)
            
            # Count unmatched predictions as false positives
            for pred_idx in range(len(predictions)):
                if pred_idx not in matched_pred_indices:
                    num_false_positives += 1
            
            # Count unmatched ground truths as false negatives
            num_unmatched_gts = len(ground_truths) - len(matched_gt_indices)
            num_false_negatives += num_unmatched_gts

    # Calculate and display metrics
    precision = num_true_positives / (num_true_positives + num_false_positives + 1e-6)
    recall = num_true_positives / (total_ground_truth_objects + 1e-6)
    f1_score = 2 * precision * recall / (precision + recall + 1e-6)
    
    print(f"\n{'='*60}")
    print(f"Evaluation Results:")
    print(f"{'='*60}")
    print(f"Detection Statistics:")
    print(f"  Raw detections:       {total_raw_detections}")
    print(f"  After conf+NMS:       {total_filtered_detections}")
    print(f"{'='*60}")
    print(f"Confusion Matrix (Detection Level):")
    print(f"  True Positives (TP):    {num_true_positives}")
    print(f"  False Positives (FP):   {num_false_positives}")
    print(f"  False Negatives (FN):   {num_false_negatives}")
    print(f"{'='*60}")
    print(f"Image Level Statistics:")
    print(f"  True Negatives (TN):    {num_true_negatives} images")
    print(f"  Total Ground Truth:     {total_ground_truth_objects} objects")
    print(f"  Total Images:           {len(all_predictions)}")
    print(f"{'='*60}")
    print(f"Performance Metrics:")
    print(f"  Precision:  {precision:.4f} ({num_true_positives}/{num_true_positives + num_false_positives})")
    print(f"  Recall:     {recall:.4f} ({num_true_positives}/{total_ground_truth_objects})")
    print(f"  F1 Score:   {f1_score:.4f}")
    print(f"{'='*60}\n")

if __name__ == "__main__":
    print(f"TensorRT version: {trt.__version__}")
    print(f"Evaluating on split: {SPLIT}")
    print(f"Confidence threshold: {CONF_THRESH}")
    print(f"IoU threshold: {IOU_THRESH}")
    print(f"NMS threshold: {NMS_THRESH}\n")
    evaluate()
