import os
import cv2
import argparse


def load_yolo_labels(label_path):
    """
    Reads a YOLO-format label file.
    Returns a list of (class_id, x_center, y_center, width, height).
    """
    boxes = []
    if not os.path.exists(label_path):
        return boxes

    with open(label_path, "r") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 5:
                continue
            class_id = int(parts[0])
            x, y, w, h = map(float, parts[1:])
            boxes.append((class_id, x, y, w, h))

    return boxes


def yolo_to_pixel(box, img_width, img_height):
    """
    Converts normalized YOLO box to pixel coordinates.
    Returns (x1, y1, x2, y2).
    """
    _, x, y, w, h = box

    x_center = x * img_width
    y_center = y * img_height
    box_width = w * img_width
    box_height = h * img_height

    x1 = int(x_center - box_width / 2)
    y1 = int(y_center - box_height / 2)
    x2 = int(x_center + box_width / 2)
    y2 = int(y_center + box_height / 2)

    return x1, y1, x2, y2


def visualize_split(dataset_root, split):
    images_dir = os.path.join(dataset_root, split, "images")
    labels_dir = os.path.join(dataset_root, split, "labels")

    if not os.path.isdir(images_dir):
        raise FileNotFoundError(f"Images directory not found: {images_dir}")

    image_files = sorted(os.listdir(images_dir))

    for img_name in image_files:
        img_path = os.path.join(images_dir, img_name)
        label_path = os.path.join(
            labels_dir, os.path.splitext(img_name)[0] + ".txt"
        )

        img = cv2.imread(img_path)
        if img is None:
            continue
            
        orig_h, orig_w = img.shape[:2]
        
        # Load boxes (normalized to original dimensions)
        boxes = load_yolo_labels(label_path)
        
        # Convert YOLO coords to pixel coords using ORIGINAL dimensions
        pixel_boxes = []
        for box in boxes:
            class_id = box[0]
            x1, y1, x2, y2 = yolo_to_pixel(box, orig_w, orig_h)
            pixel_boxes.append((class_id, x1, y1, x2, y2))
        
        # Now resize image to 640x640
        display_size = 640
        img = cv2.resize(img, (display_size, display_size), interpolation=cv2.INTER_LINEAR)
        
        # Scale the pixel coordinates to match resized image
        scale_x = display_size / orig_w
        scale_y = display_size / orig_h
        
        for class_id, x1, y1, x2, y2 in pixel_boxes:
            # Scale coordinates
            x1_scaled = int(x1 * scale_x)
            y1_scaled = int(y1 * scale_y)
            x2_scaled = int(x2 * scale_x)
            y2_scaled = int(y2 * scale_y)

            cv2.rectangle(img, (x1_scaled, y1_scaled), (x2_scaled, y2_scaled), (0, 255, 0), 2)
            cv2.putText(
                img,
                f"ID {class_id}",
                (x1_scaled, max(y1_scaled - 5, 15)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        cv2.imshow(f"{split} - YOLO Boxes", img)
        key = cv2.waitKey(0)

        if key == 27:
            break

    cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description="Visualize YOLO bounding boxes with OpenCV")
    parser.add_argument(
        "--dataset",
        required=True,
        help="Path to dataset root (contains train/valid/test folders)",
    )
    parser.add_argument(
        "--split",
        default="train",
        choices=["train", "valid", "test"],
        help="Dataset split to visualize",
    )

    args = parser.parse_args()
    visualize_split(args.dataset, args.split)


if __name__ == "__main__":
    main()

