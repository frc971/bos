import os
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit  # initializes CUDA driver automatically
from glob import glob
from tqdm import tqdm

# -------------------------------
# Configuration
# -------------------------------
ENGINE_PATH = "/bos/src/yolo/thirdYOLO.engine"
DATASET_ROOT = "/home/nvidia/Documents/gamepiece-data"  # Root directory containing train/valid/test folders
SPLIT = "train"  # Which split to evaluate: "train", "valid", or "test"
IMG_SIZE = 640  # model input size

CONF_THRESH = 0.25
IOU_THRESH = 0.45

# Visualization settings
SAVE_EXAMPLES = True  # Whether to save example images
EXAMPLES_PER_TYPE = 5  # Number of examples to save per detection type
OUTPUT_DIR = "test_images"  # Directory to save visualization images

# YOLOv11 dataset structure:
# dataset_root/
#   ├── train/
#   │   ├── images/
#   │   └── labels/
#   ├── valid/
#   │   ├── images/
#   │   └── labels/
#   ├── test/
#   │   ├── images/
#   │   └── labels/
#   └── data.yaml

DATASET_DIR = os.path.join(DATASET_ROOT, SPLIT, "images")
LABEL_DIR = os.path.join(DATASET_ROOT, SPLIT, "labels")

# -------------------------------
# Helper functions
# -------------------------------

def letterbox(img, new_shape=640, color=(114, 114, 114)):
    """Resize image to fit model input (same as YOLOv5 preprocessing)."""
    shape = img.shape[:2]  # current shape [height, width]
    r = min(new_shape / shape[0], new_shape / shape[1])
    new_unpad = (int(round(shape[1] * r)), int(round(shape[0] * r)))
    dw, dh = new_shape - new_unpad[0], new_shape - new_unpad[1]
    dw /= 2
    dh /= 2

    img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, r, (dw, dh)


def iou(box1, box2):
    """Compute IoU between two boxes [x1, y1, x2, y2]."""
    inter_x1 = max(box1[0], box2[0])
    inter_y1 = max(box1[1], box2[1])
    inter_x2 = min(box1[2], box2[2])
    inter_y2 = min(box1[3], box2[3])
    inter_area = max(inter_x2 - inter_x1, 0) * max(inter_y2 - inter_y1, 0)
    box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
    box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
    return inter_area / (box1_area + box2_area - inter_area + 1e-6)


def draw_boxes(img, predictions, ground_truths, matched_preds, matched_gts, fp_preds):
    """
    Draw bounding boxes on image with color coding:
    - Green: True Positives (matched predictions)
    - Red: False Positives (unmatched predictions)
    - Blue: False Negatives (unmatched ground truths)
    """
    img_vis = img.copy()
    h, w = img.shape[:2]
    
    # Draw False Negatives (unmatched ground truths) in BLUE
    for gt_idx, gt in enumerate(ground_truths):
        if gt_idx not in matched_gts:
            x1, y1, x2, y2, cls = gt
            x1, y1, x2, y2 = int(x1 * w), int(y1 * h), int(x2 * w), int(y2 * h)
            cv2.rectangle(img_vis, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(img_vis, f'FN cls:{int(cls)}', (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    
    # Draw False Positives (unmatched predictions) in RED
    for pred_idx in fp_preds:
        pred = predictions[pred_idx]
        x1, y1, x2, y2, conf, cls = pred
        x1, y1, x2, y2 = int(x1 * w), int(y1 * h), int(x2 * w), int(y2 * h)
        cv2.rectangle(img_vis, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.putText(img_vis, f'FP {conf:.2f} cls:{cls}', (x1, y1 - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # Draw True Positives (matched predictions) in GREEN
    for pred_idx in matched_preds:
        pred = predictions[pred_idx]
        x1, y1, x2, y2, conf, cls = pred
        x1, y1, x2, y2 = int(x1 * w), int(y1 * h), int(x2 * w), int(y2 * h)
        cv2.rectangle(img_vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(img_vis, f'TP {conf:.2f} cls:{cls}', (x1, y1 - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return img_vis


# -------------------------------
# TensorRT Inference Wrapper
# -------------------------------

class TRTInfer:
    def __init__(self, engine_path):
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        
        if self.engine is None:
            print("Engine is nullptr, tried to load it from " + engine_path)
            print("Engine file exists: " + ("true" if os.path.exists(engine_path) else "false"))
            raise RuntimeError("Failed to load TensorRT engine")
        
        self.context = self.engine.create_execution_context()
        
        # Use TensorRT 10.x API
        self.input_names = []
        self.output_names = []
        
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self.input_names.append(name)
            else:
                self.output_names.append(name)
        
        print("Input tensors:", self.input_names)
        print("Output tensors:", self.output_names)
        
        # Assume first input and output
        self.input_name = self.input_names[0]
        self.output_name = self.output_names[0]
        
        self.stream = cuda.Stream()
        
        # Get shapes using new API
        self.input_shape = self.engine.get_tensor_shape(self.input_name)
        self.output_shape = self.engine.get_tensor_shape(self.output_name)
        
        print(f"Input shape: {self.input_shape}")
        print(f"Output shape: {self.output_shape}")
        
        # Calculate sizes
        self.input_nbytes = trt.volume(self.input_shape) * np.dtype(np.float32).itemsize
        self.output_nbytes = trt.volume(self.output_shape) * np.dtype(np.float32).itemsize
        
        # Allocate device memory
        self.d_input = cuda.mem_alloc(self.input_nbytes)
        self.d_output = cuda.mem_alloc(self.output_nbytes)

    def infer(self, img, orig_shape):
        """
        Returns predictions and metadata needed for coordinate conversion.
        """
        h_orig, w_orig = orig_shape
        
        # Preprocess
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img, ratio, (dw, dh) = letterbox(img, IMG_SIZE)
        img = img.transpose(2, 0, 1)
        img = np.expand_dims(img, 0)
        img = np.ascontiguousarray(img, dtype=np.float32) / 255.0

        # Copy input to device
        cuda.memcpy_htod_async(self.d_input, img, self.stream)
        
        # Set tensor addresses using new API
        self.context.set_tensor_address(self.input_name, int(self.d_input))
        self.context.set_tensor_address(self.output_name, int(self.d_output))
        
        # Execute inference
        self.context.execute_async_v3(stream_handle=self.stream.handle)
        
        # Copy output back to host
        output = np.empty(self.output_shape, dtype=np.float32)
        cuda.memcpy_dtoh_async(output, self.d_output, self.stream)
        self.stream.synchronize()

        return output, ratio, (dw, dh), (h_orig, w_orig)


def process_predictions(output, ratio, pad, orig_shape, conf_thresh=0.25):
    """
    Process model output with NMS already applied.
    Assumes output format: [num_detections, 6] where each row is [x1, y1, x2, y2, conf, cls]
    or [1, num_detections, 6] or [num_detections, 7] (with batch index)
    
    Converts coordinates back to original image space and filters by confidence.
    """
    dw, dh = pad
    h_orig, w_orig = orig_shape
    
    # Handle different output shapes
    if len(output.shape) == 3:
        output = output[0]  # Remove batch dimension if present
    
    if output.shape[0] == 0:
        return []
    
    # Debug: print first few detections
    # print(f"  Raw output shape: {output.shape}")
    # print(f"  First detection (raw): {output[0] if len(output) > 0 else 'None'}")
    
    predictions = []
    for detection in output:
        # Handle different formats
        if len(detection) >= 6:
            # Format: [x1, y1, x2, y2, conf, cls] or [batch, x1, y1, x2, y2, conf, cls]
            if len(detection) == 7:
                _, x1, y1, x2, y2, conf, cls = detection
            else:
                x1, y1, x2, y2, conf, cls = detection[:6]
            
            # Filter by confidence
            if conf < conf_thresh:
                continue
            
            # Convert from letterbox coordinates to original image coordinates
            # Remove padding
            x1 = (x1 - dw) / ratio
            y1 = (y1 - dh) / ratio
            x2 = (x2 - dw) / ratio
            y2 = (y2 - dh) / ratio
            
            # Clip to image bounds
            x1 = max(0, min(x1, w_orig))
            y1 = max(0, min(y1, h_orig))
            x2 = max(0, min(x2, w_orig))
            y2 = max(0, min(y2, h_orig))
            
            # Convert to normalized coordinates [0, 1] to match ground truth
            x1_norm = x1 / w_orig
            y1_norm = y1 / h_orig
            x2_norm = x2 / w_orig
            y2_norm = y2 / h_orig
            
            predictions.append([x1_norm, y1_norm, x2_norm, y2_norm, conf, int(cls)])
    
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
        print(f"Expected YOLOv11 structure: {DATASET_ROOT}/{SPLIT}/images/")
        exit(1)
    
    if not os.path.exists(LABEL_DIR):
        print(f"ERROR: Label directory not found: {LABEL_DIR}")
        print(f"Expected YOLOv11 structure: {DATASET_ROOT}/{SPLIT}/labels/")
        exit(1)
    
    # Create output directory for visualizations
    if SAVE_EXAMPLES:
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        for subdir in ['true_positive', 'false_positive', 'false_negative', 'true_negative']:
            os.makedirs(os.path.join(OUTPUT_DIR, subdir), exist_ok=True)
        print(f"Output directory created: {OUTPUT_DIR}\n")
    
    # Check for data.yaml
    data_yaml_path = os.path.join(DATASET_ROOT, "data.yaml")
    if os.path.exists(data_yaml_path):
        print(f"Found data.yaml at: {data_yaml_path}")
        try:
            with open(data_yaml_path, 'r') as f:
                print("data.yaml contents:")
                print(f.read())
        except:
            pass
    else:
        print(f"Warning: data.yaml not found at {data_yaml_path}")
    
    print(f"\nDataset structure:")
    print(f"  Root: {DATASET_ROOT}")
    print(f"  Split: {SPLIT}")
    print(f"  Images: {DATASET_DIR}")
    print(f"  Labels: {LABEL_DIR}\n")
    
    inferer = TRTInfer(ENGINE_PATH)
    print("Inferer created\n")
    img_files = sorted(glob(os.path.join(DATASET_DIR, "*.jpg")) + glob(os.path.join(DATASET_DIR, "*.png")))
    if len(img_files) == 0:
        print("Img_files is empty!")
        exit(0)

    print(f"Found {len(img_files)} images\n")

    all_preds, all_gts = [], []
    total_raw_detections = 0
    total_filtered_detections = 0
    
    # Store image data for visualization
    image_data = []  # List of (img_path, img, preds, gts)
    
    for idx, img_path in enumerate(tqdm(img_files, desc="Evaluating")):
        label_path = os.path.join(LABEL_DIR, os.path.basename(img_path).rsplit(".", 1)[0] + ".txt")
        if not os.path.exists(label_path):
            print(f"Label path doesn't exist: {label_path}")
            continue

        img = cv2.imread(img_path)
        if img is None:
            print(f"Failed to read image: {img_path}")
            continue
        
        orig_shape = img.shape[:2]
        output, ratio, pad, orig = inferer.infer(img, orig_shape)
        
        # Count raw detections
        if len(output.shape) == 3:
            raw_count = output.shape[1]
        else:
            raw_count = output.shape[0]
        total_raw_detections += raw_count
        
        # Process predictions
        preds = process_predictions(output, ratio, pad, orig, CONF_THRESH)
        total_filtered_detections += len(preds)
        
    #    if idx < 3:  # Debug first 3 images
            # print(f"\nImage {idx}: {os.path.basename(img_path)}")
            # print(f"  Original shape: {orig_shape}")
            # print(f"  Raw detections: {raw_count}")
            # print(f"  Filtered detections: {len(preds)}")
    #        if len(preds) > 0:
                # print(f"  First prediction: {preds[0]}")
        
        all_preds.append(preds)

        # Load ground truth in YOLO format (normalized coordinates)
        gts = []
        with open(label_path) as f:
            for line in f.readlines():
                parts = line.strip().split()
                if len(parts) == 5:
                    cls, x_center, y_center, w, h = map(float, parts)
                    # Convert from center format to corner format (normalized)
                    x1 = x_center - w / 2
                    y1 = y_center - h / 2
                    x2 = x_center + w / 2
                    y2 = y_center + h / 2
                    gts.append([x1, y1, x2, y2, cls])
        all_gts.append(gts)
        
        # Store image data for later visualization
        image_data.append((img_path, img, preds, gts))

    # print(f"\n{'='*60}")
    # print(f"Detection Summary:")
    # print(f"  Total raw detections: {total_raw_detections}")
    # print(f"  Total filtered detections (conf > {CONF_THRESH}): {total_filtered_detections}")
    # print(f"  Average detections per image: {total_filtered_detections / len(img_files):.2f}")
    # print(f"{'='*60}\n")

    # Compute precision/recall (mAP@0.5)
    tp, fp, fn = 0, 0, 0
    total_gt = sum(len(g) for g in all_gts)
    
    # Track examples for each category - store with counts for prioritization
    tp_candidates = []  # Images with ONLY true positives (all preds matched, all GTs matched)
    fp_candidates = []  # Images with ONLY false positives (preds but no GT, or unmatched preds)
    fn_candidates = []  # Images with ONLY false negatives (GT but no preds, or unmatched GTs)
    tn_candidates = []  # Images with true negatives (no predictions, no GT)
    
    # Store detailed results per image
    image_results = []
    
    for idx, (preds, gts) in enumerate(zip(all_preds, all_gts)):
        matched_gt = set()
        matched_pred = set()
        fp_preds = set()
        
        # Match predictions to ground truth
        for pred_idx, pred in enumerate(preds):
            p_box = pred[:4]  # [x1, y1, x2, y2] in normalized coords
            best_iou = 0
            best_gt_idx = -1
            
            for gt_idx, gt in enumerate(gts):
                if gt_idx in matched_gt:
                    continue
                g_box = gt[:4]  # [x1, y1, x2, y2] in normalized coords
                curr_iou = iou(p_box, g_box)
                if curr_iou > best_iou:
                    best_iou = curr_iou
                    best_gt_idx = gt_idx
            
            if best_iou > 0.5 and best_gt_idx != -1:
                tp += 1
                matched_gt.add(best_gt_idx)
                matched_pred.add(pred_idx)
            else:
                fp += 1
                fp_preds.add(pred_idx)
        
        # Count false negatives (unmatched ground truth)
        num_fn = len(gts) - len(matched_gt)
        fn += num_fn
        
        # Count each type for this image
        num_tp = len(matched_pred)
        num_fp = len(fp_preds)
        
        image_results.append({
            'idx': idx,
            'matched_pred': matched_pred,
            'matched_gt': matched_gt,
            'fp_preds': fp_preds,
            'num_tp': num_tp,
            'num_fp': num_fp,
            'num_fn': num_fn
        })
        
        # Categorize image by predominant type
        # Priority: TN > pure TP > pure FP > pure FN > mixed
        if len(preds) == 0 and len(gts) == 0:
            # True Negative: no predictions, no ground truth
            tn_candidates.append((idx, 1))
        elif num_tp > 0 and num_fp == 0 and num_fn == 0:
            # Pure True Positive: all predictions matched, all GTs matched
            tp_candidates.append((idx, num_tp))
        elif num_fp > 0 and num_tp == 0 and num_fn == 0:
            # Pure False Positive: only wrong predictions, no GT
            fp_candidates.append((idx, num_fp))
        elif num_fn > 0 and num_tp == 0 and num_fp == 0:
            # Pure False Negative: only missed detections, no predictions
            fn_candidates.append((idx, num_fn))
        else:
            # Mixed case - categorize by what dominates
            if num_tp > num_fp and num_tp > num_fn:
                tp_candidates.append((idx, num_tp))
            elif num_fp > num_tp and num_fp > num_fn:
                fp_candidates.append((idx, num_fp))
            elif num_fn > 0:
                fn_candidates.append((idx, num_fn))
        
        # Debug first few images
    #    if idx < 3:
            # print(f"Image {idx}: {len(preds)} preds, {len(gts)} GTs, TP:{num_tp} FP:{num_fp} FN:{num_fn}")
    
    # Sort candidates by count (descending) and select top examples
    tp_candidates.sort(key=lambda x: x[1], reverse=True)
    fp_candidates.sort(key=lambda x: x[1], reverse=True)
    fn_candidates.sort(key=lambda x: x[1], reverse=True)
    tn_candidates.sort(key=lambda x: x[1], reverse=True)
    
    tp_examples = [idx for idx, _ in tp_candidates[:EXAMPLES_PER_TYPE]]
    fp_examples = [idx for idx, _ in fp_candidates[:EXAMPLES_PER_TYPE]]
    fn_examples = [idx for idx, _ in fn_candidates[:EXAMPLES_PER_TYPE]]
    tn_examples = [idx for idx, _ in tn_candidates[:EXAMPLES_PER_TYPE]]
    
    # print(f"\nFound {len(tp_candidates)} TP images, {len(fp_candidates)} FP images, "
    #      f"{len(fn_candidates)} FN images, {len(tn_candidates)} TN images")
    
    # Save example images
    if SAVE_EXAMPLES:
        print(f"\nSaving example images to {OUTPUT_DIR}/...")
        
        for category, examples in [('true_positive', tp_examples), 
                                     ('false_positive', fp_examples),
                                     ('false_negative', fn_examples),
                                     ('true_negative', tn_examples)]:
            for i, img_idx in enumerate(examples):
                img_path, img, preds, gts = image_data[img_idx]
                result = image_results[img_idx]
                
                if category == 'true_negative':
                    # TN: just save the original image with a label
                    img_vis = img.copy()
                    cv2.putText(img_vis, 'True Negative (No detections, No GT)', 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    # Draw boxes for other categories
                    img_vis = draw_boxes(img, preds, gts, 
                                        result['matched_pred'], 
                                        result['matched_gt'], 
                                        result['fp_preds'])
                
                output_path = os.path.join(OUTPUT_DIR, category, 
                                          f"{i:03d}_{os.path.basename(img_path)}")
                cv2.imwrite(output_path, img_vis)
        
        print(f"Saved {len(tp_examples)} TP, {len(fp_examples)} FP, "
              f"{len(fn_examples)} FN, {len(tn_examples)} TN examples")

    precision = tp / (tp + fp + 1e-6)
    recall = tp / (total_gt + 1e-6)
    f1 = 2 * precision * recall / (precision + recall + 1e-6)
    
    print(f"\n{'='*60}")
    print(f"Evaluation Results:")
    print(f"{'='*60}")
    print(f"True Positives (TP):   {tp}")
    print(f"False Positives (FP):  {fp}")
    print(f"False Negatives (FN):  {fn}")
    print(f"Total Ground Truth:    {total_gt}")
    print(f"{'='*60}")
    print(f"Precision:  {precision:.3f} ({tp}/{tp + fp})")
    print(f"Recall:     {recall:.3f} ({tp}/{total_gt})")
    print(f"F1 Score:   {f1:.3f}")
    print(f"{'='*60}\n")

if __name__ == "__main__":
    print(f"TensorRT version: {trt.__version__}")
    print(f"Evaluating on split: {SPLIT}")
    print(f"Confidence threshold: {CONF_THRESH}")
    print(f"IoU threshold: {IOU_THRESH}\n")
    evaluate()
