from PIL import Image
import numpy as np
import torch
import open_clip
import constants as constants
from pathlib import Path
import cv2

class CLIPReranker:
    def __init__(self, device="cuda"):
        self.device = device

        self.model, _, self.preprocess = open_clip.create_model_and_transforms(
            "ViT-B-32", pretrained="openai"
        )
        self.model.to(self.device)
        self.model.eval()

        self.positive_prompts = [
            "a " + constants.OBJECT
        ]

        self.negative_prompts = [
            "a space heater",
            "a rock",
            "a stone",
            "a metal cylinder",
            "a round household object"
            "a tree"
        ]

        with torch.no_grad():
            self.text_features = self._encode_text(
                self.positive_prompts + self.negative_prompts
            )

    def _encode_text(self, prompts):
        tokens = open_clip.tokenize(prompts).to(self.device)
        text_features = self.model.encode_text(tokens)
        return text_features / text_features.norm(dim=-1, keepdim=True)

    def score_crop(self, image_crop: Image.Image):
        image_tensor = self.preprocess(image_crop).unsqueeze(0).to(self.device)

        with torch.no_grad():
            image_features = self.model.encode_image(image_tensor)
            image_features /= image_features.norm(dim=-1, keepdim=True)

            logits = (image_features @ self.text_features.T).squeeze(0)
            probs = logits.softmax(dim=0)

        pos_count = len(self.positive_prompts)
        pos_score = probs[:pos_count].max().item()
        neg_score = probs[pos_count:].max().item()

        return pos_score, neg_score

    def is_positive(self, image_crop):
        pos, neg = self.score_crop(image_crop)
        return pos > 0 and pos >= neg
    
def show_blocking(pil_img, title="image"):
    img = np.array(pil_img)[:, :, ::-1]  # RGB → BGR
    cv2.imshow(title, img)
    cv2.waitKey(0)
    cv2.destroyWindow(title)
    
def yolo_to_xyxy(label, img_w, img_h):
    _, cx, cy, w, h = label
    cx *= img_w
    cy *= img_h
    w *= img_w
    h *= img_h

    x1 = int(cx - w / 2)
    y1 = int(cy - h / 2)
    x2 = int(cx + w / 2)
    y2 = int(cy + h / 2)

    x1 = max(0, min(x1, img_w - 1))
    y1 = max(0, min(y1, img_h - 1))
    x2 = max(1, min(x2, img_w))
    y2 = max(1, min(y2, img_h))

    return x1, y1, x2, y2

def clean_split(reranker: CLIPReranker, images_dir: Path, labels_dir: Path):
    for label_path in labels_dir.glob("*.txt"):
        image_path = images_dir / (label_path.stem + ".jpg")
        if not image_path.exists():
            continue

        image = Image.open(image_path).convert("RGB")
        img_w, img_h = image.size

        kept_labels = []

        with open(label_path, "r") as f:
            lines = f.readlines()

        for line in lines:
            parts = list(map(float, line.strip().split()))
            if len(parts) != 5:
                continue

            x1, y1, x2, y2 = yolo_to_xyxy(parts, img_w, img_h)

            if x2 <= x1 or y2 <= y1:
                continue

            crop = image.crop((x1, y1, x2, y2))

            if reranker.is_positive(crop):
                print("keeping")
                kept_labels.append(line)
            else:
                print("removing")
                show_blocking(crop)

        # Overwrite label file with cleaned annotations
        with open(label_path, "w") as f:
            f.writelines(kept_labels)

def main():
    reranker = CLIPReranker()
    for split in ["train", "val"]:
        dir = constants.OUTPUT_FOLDER + "/" + split
        clean_split(reranker=reranker, images_dir=Path(dir + "/images"), labels_dir=Path(dir + "/labels"))

if __name__=="__main__":
    main()