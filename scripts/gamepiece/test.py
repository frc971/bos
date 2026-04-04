import os
import cv2
import numpy as np
import onnxruntime as ort
from glob import glob
from tqdm import tqdm

# Configuration
MODEL_PATH = "/home/yasen/Training/runs/detect/train3/weights/best.onnx"
DATASET_ROOT = "/mnt/c/Users/Yasen/Documents/gamepiece-data"
SPLIT = "test"
IMG_SIZE = 640
CONF_THRESH = 0.25
IOU_THRESH = 0.5

DATASET_DIR = os.path.join(DATASET_ROOT, SPLIT, "images")
LABEL_DIR = os.path.join(DATASET_ROOT, SPLIT, "labels")


def letterbox(img, new_shape=640):
    """Resize and pad image to square."""
    h, w = img.shape[:2]
    r = min(new_shape / h, new_shape / w)
    new_w, new_h = int(round(w * r)), int(round(h * r))
    dw, dh = (new_shape - new_w) / 2, (new_shape - new_h) / 2
    
    img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return img, r, (dw, dh)


def compute_iou(box1, box2):
    """Compute IoU between two boxes [x1, y1, x2, y2]."""
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    return inter / (area1 + area2 - inter + 1e-6)

def nms(boxes, scores, iou_thresh):
    """Non-Maximum Suppression."""
    indices = []
    boxes = np.array(boxes)
    scores = np.array(scores)
    order = scores.argsort()[::-1]

    while order.size > 0:
        i = order[0]
        indices.append(i)
        if order.size == 1:
            break
        ious = np.array([compute_iou(boxes[i], boxes[j]) for j in order[1:]])
        order = order[1:][ious < iou_thresh]

    return indices


def preprocess(img):
    """Preprocess image for ONNX model."""
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    img, ratio, padding = letterbox(img, IMG_SIZE)
    img = img.transpose(2, 0, 1).astype(np.float32) / 255.0
    img = np.expand_dims(img, 0)
    return img, ratio, padding


def postprocess(output, ratio, padding, orig_shape, conf_thresh, iou_thresh=0.5):
    """Convert model output to normalized coordinates."""
    dw, dh = padding
    h_orig, w_orig = orig_shape

    if len(output.shape) == 3:
        output = output[0]
    if output.shape[0] == 6 and output.shape[1] != 6:
        output = output.T

    if output.shape[0] == 0 or output.shape[1] != 6:
        return []

    for pred in output:
        cx, cy, w, h, cls, conf = pred
        
        if conf < conf_thresh:
            continue
        
        # Convert from center format to corner format
        x1 = cx - w / 2
        y1 = cy - h / 2
        x2 = cx + w / 2
        y2 = cy + h / 2
        
        # Convert to original image coordinates
        x1 = np.clip((x1 - dw) / ratio, 0, w_orig)
        y1 = np.clip((y1 - dh) / ratio, 0, h_orig)
        x2 = np.clip((x2 - dw) / ratio, 0, w_orig)
        y2 = np.clip((y2 - dh) / ratio, 0, h_orig)
        
        boxes.append([x1, y1, x2, y2])
        scores.append(float(conf))
        classes.append(int(cls))

    # Apply NMS per class
    final_preds = []
    for c in set(classes):
        cls_idxs = [i for i, cl in enumerate(classes) if cl == c]
        cls_boxes = [boxes[i] for i in cls_idxs]
        cls_scores = [scores[i] for i in cls_idxs]

        keep = nms(cls_boxes, cls_scores, iou_thresh)
        for i in keep:
            x1, y1, x2, y2 = cls_boxes[i]
            conf = cls_scores[i]
            final_preds.append([
                x1 / w_orig, y1 / h_orig, x2 / w_orig, y2 / h_orig, conf, c
            ])
    
    return final_preds

def load_ground_truth(label_path):
    """Load YOLO format labels."""
    ground_truths = []
    with open(label_path) as f:
        for line in f:
            cls, xc, yc, w, h = map(float, line.strip().split())
            x1, y1 = xc - w/2, yc - h/2
            x2, y2 = xc + w/2, yc + h/2
            ground_truths.append([x1, y1, x2, y2, cls])
    return ground_truths


def evaluate():
    # Verify paths
    for path, name in [(DATASET_ROOT, "Dataset root"), (DATASET_DIR, "Images"), (LABEL_DIR, "Labels")]:
        if not os.path.exists(path):
            print(f"ERROR: {name} not found: {path}")
            exit(1)
    
    # Load ONNX model
    session = ort.InferenceSession(MODEL_PATH, providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name
    
    # Get image files
    img_files = sorted(glob(os.path.join(DATASET_DIR, "*.jpg")) + 
                      glob(os.path.join(DATASET_DIR, "*.png")))
    
    if not img_files:
        print("No images found!")
        exit(1)
    
    print(f"Found {len(img_files)} images")
    print(f"Model: {MODEL_PATH}")
    print(f"Confidence threshold: {CONF_THRESH}\n")
    
    # Evaluation metrics
    tp, fp, fn = 0, 0, 0
    total_gt = 0
    
    for img_path in tqdm(img_files, desc="Evaluating"):
        label_path = os.path.join(LABEL_DIR, os.path.splitext(os.path.basename(img_path))[0] + ".txt")
        
        if not os.path.exists(label_path):
            continue
        
        # Load and process image
        img = cv2.imread(img_path)
        if img is None:
            continue
        
        orig_shape = img.shape[:2]
        input_img, ratio, padding = preprocess(img)
        
        # Run inference
        output = session.run(None, {input_name: input_img})[0]
        preds = postprocess(output, ratio, padding, orig_shape, CONF_THRESH)
        
        # Load ground truth
        ground_truths = load_ground_truth(label_path)
        total_gt += len(ground_truths)
        
        # Match predictions to ground truth
        matched_gt = set()
        matched_pred = set()
        
        for pred_idx, pred in enumerate(preds):
            best_iou = 0
            best_gt_idx = -1
            
            for gt_idx, gt in enumerate(ground_truths):
                if gt_idx in matched_gt:
                    continue
                curr_iou = compute_iou(pred[:4], gt[:4])
                if curr_iou > best_iou:
                    best_iou = curr_iou
                    best_gt_idx = gt_idx
            
            if best_iou > IOU_THRESH and best_gt_idx != -1:
                tp += 1
                matched_gt.add(best_gt_idx)
                matched_pred.add(pred_idx)
            else:
                fp += 1
        
        # Count false negatives
        fn += len(ground_truths) - len(matched_gt)
    
    # Calculate metrics
    precision = tp / (tp + fp + 1e-6)
    recall = tp / (total_gt + 1e-6)
    f1 = 2 * precision * recall / (precision + recall + 1e-6)
    
    # Print results
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
    evaluate()
