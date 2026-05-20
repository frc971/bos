import shutil
from pathlib import Path
import yaml

SPLITS = ["train", "valid", "test"]
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def load_data_yaml(dataset_root: Path) -> dict:
    data_yaml = dataset_root / "data.yaml"
    if not data_yaml.exists():
        raise FileNotFoundError(f"Missing data.yaml in target dataset: {dataset_root}")
    with open(data_yaml, "r") as f:
        return yaml.safe_load(f)


def move_split(source_root: Path, target_root: Path, split: str):
    src_img_dir = source_root / split / "images"
    src_lbl_dir = source_root / split / "labels"

    tgt_img_dir = target_root / split / "images"
    tgt_lbl_dir = target_root / split / "labels"

    if not src_img_dir.exists():
        print(f"[WARN] Source split missing: {src_img_dir}")
        return

    tgt_img_dir.mkdir(parents=True, exist_ok=True)
    tgt_lbl_dir.mkdir(parents=True, exist_ok=True)

    for img_path in src_img_dir.iterdir():
        if img_path.suffix.lower() not in IMAGE_EXTS:
            continue

        label_path = src_lbl_dir / (img_path.stem + ".txt")

        if not label_path.exists():
            print(f"[SKIP] Missing label for {img_path.name}")
            continue

        shutil.move(str(img_path), tgt_img_dir / img_path.name)
        shutil.move(str(label_path), tgt_lbl_dir / label_path.name)


def move_dataset(source_dataset: str, target_dataset: str):
    source_root = Path(source_dataset).resolve()
    target_root = Path(target_dataset).resolve()

    # Enforce YOLOv11 target dataset by requiring data.yaml
    load_data_yaml(target_root)

    for split in SPLITS:
        move_split(source_root, target_root, split)

    print("Dataset move complete.")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Move YOLOv11 images+labels into another dataset")
    parser.add_argument("source", help="Source YOLO dataset root")
    parser.add_argument("target", help="Target YOLOv11 dataset root (must contain data.yaml)")
    args = parser.parse_args()

    move_dataset(args.source, args.target)

