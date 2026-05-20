import shutil
from pathlib import Path
import yaml

SPLITS = ["train", "valid", "test"]
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def require_data_yaml(dataset_root: Path):
    data_yaml = dataset_root / "data.yaml"
    if not data_yaml.exists():
        raise FileNotFoundError(
            f"data.yaml not found in {dataset_root}. "
            "This script only supports YOLOv11 exports."
        )
    with open(data_yaml, "r") as f:
        yaml.safe_load(f)


def extract_images(dataset_root: Path, output_dir: Path, move: bool):
    output_dir.mkdir(parents=True, exist_ok=True)

    for split in SPLITS:
        img_dir = dataset_root / split / "images"
        if not img_dir.exists():
            continue

        for img_path in img_dir.iterdir():
            if img_path.suffix.lower() not in IMAGE_EXTS:
                continue

            # Prefix filename with split to avoid collisions
            new_name = f"{split}_{img_path.name}"
            dst = output_dir / new_name

            if move:
                shutil.move(img_path, dst)
            else:
                shutil.copy2(img_path, dst)


def main(source_dataset: str, output_folder: str, move: bool):
    dataset_root = Path(source_dataset).resolve()
    output_dir = Path(output_folder).resolve()

    require_data_yaml(dataset_root)
    extract_images(dataset_root, output_dir, move)

    print("Image extraction complete.")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Extract all images from a YOLOv11 dataset into a single folder"
    )
    parser.add_argument("dataset", help="YOLOv11 dataset root (must contain data.yaml)")
    parser.add_argument("output", help="Output directory for extracted images")
    parser.add_argument(
        "--move",
        action="store_true",
        help="Move images instead of copying (destructive)",
    )

    args = parser.parse_args()
    main(args.dataset, args.output, args.move)

