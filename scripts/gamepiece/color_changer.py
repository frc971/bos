#!/usr/bin/env python3
"""
Color replacement script that converts shades of a source color to corresponding
shades of a target color in an image, directory of images, or YOLO datasets.

Usage:
    python3 color_replace.py input.png --src=red --target=yellow --output=output.png
    python3 color_replace.py input_dir/ --src=red --target=yellow --output=output_dir/
    python3 color_replace.py dataset/ --src=red --target=yellow --output=new_dataset/ --yolo
"""

import argparse
import numpy as np
from PIL import Image
import colorsys
import os
from pathlib import Path
import shutil
import yaml


def show_comparison(original_img, transformed_img):
    """Display original and transformed images side-by-side."""
    try:
        import matplotlib.pyplot as plt
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
        
        ax1.imshow(original_img)
        ax1.set_title('Original', fontsize=14, fontweight='bold')
        ax1.axis('off')
        
        ax2.imshow(transformed_img)
        ax2.set_title('Transformed', fontsize=14, fontweight='bold')
        ax2.axis('off')
        
        plt.tight_layout()
        plt.show()
        
    except ImportError:
        print("\nWarning: matplotlib not installed. Cannot display comparison.")
        print("Install with: pip install matplotlib")


def parse_color(color_str):
    """Parse color string to RGB tuple (0-255 range)."""
    color_map = {
        'red': (255, 0, 0),
        'green': (0, 255, 0),
        'blue': (0, 0, 255),
        'yellow': (255, 255, 0),
        'cyan': (0, 255, 255),
        'magenta': (255, 0, 255),
        'white': (255, 255, 255),
        'black': (0, 0, 0),
        'orange': (255, 165, 0),
        'purple': (128, 0, 128),
        'pink': (255, 192, 203),
        'brown': (165, 42, 42),
        'gray': (128, 128, 128),
        'grey': (128, 128, 128),
    }
    
    color_str = color_str.lower()
    
    # Check if it's a named color
    if color_str in color_map:
        return color_map[color_str]
    
    # Check if it's a hex color
    if color_str.startswith('#'):
        color_str = color_str[1:]
    if len(color_str) == 6:
        try:
            return tuple(int(color_str[i:i+2], 16) for i in (0, 2, 4))
        except ValueError:
            pass
    
    # Try parsing as RGB tuple
    if ',' in color_str:
        try:
            rgb = tuple(int(x.strip()) for x in color_str.split(','))
            if len(rgb) == 3 and all(0 <= x <= 255 for x in rgb):
                return rgb
        except ValueError:
            pass
    
    raise ValueError(f"Invalid color format: {color_str}")


def rgb_to_hsv(r, g, b):
    """Convert RGB (0-255) to HSV (H: 0-360, S: 0-1, V: 0-1)."""
    return colorsys.rgb_to_hsv(r/255, g/255, b/255)


def hsv_to_rgb(h, s, v):
    """Convert HSV (H: 0-1, S: 0-1, V: 0-1) to RGB (0-255)."""
    r, g, b = colorsys.hsv_to_rgb(h, s, v)
    return int(r * 255), int(g * 255), int(b * 255)


def replace_color(img_array, src_rgb, target_rgb, tolerance=30):
    """
    Replace shades of source color with corresponding shades of target color.
    
    Args:
        img_array: numpy array of image (H, W, 3 or 4)
        src_rgb: source color as RGB tuple (0-255)
        target_rgb: target color as RGB tuple (0-255)
        tolerance: color matching tolerance (0-100), higher = more permissive
    
    Returns:
        Modified image array
    """
    # Convert to float for processing
    result = img_array.astype(float)
    
    # Get HSV values for source and target
    src_h, src_s, src_v = rgb_to_hsv(*src_rgb)
    target_h, target_s, target_v = rgb_to_hsv(*target_rgb)
    
    # Convert image to HSV - vectorized operation
    rgb_img = result[:, :, :3] / 255.0
    
    # Vectorized RGB to HSV conversion
    r, g, b = rgb_img[:, :, 0], rgb_img[:, :, 1], rgb_img[:, :, 2]
    maxc = np.maximum(np.maximum(r, g), b)
    minc = np.minimum(np.minimum(r, g), b)
    v = maxc
    
    deltac = maxc - minc
    s = np.where(maxc != 0, deltac / maxc, 0)
    
    # Calculate hue
    h = np.zeros_like(maxc)
    
    # Red is max
    mask_r = (maxc == r) & (deltac != 0)
    h[mask_r] = ((g[mask_r] - b[mask_r]) / deltac[mask_r]) % 6
    
    # Green is max
    mask_g = (maxc == g) & (deltac != 0)
    h[mask_g] = ((b[mask_g] - r[mask_g]) / deltac[mask_g]) + 2
    
    # Blue is max
    mask_b = (maxc == b) & (deltac != 0)
    h[mask_b] = ((r[mask_b] - g[mask_b]) / deltac[mask_b]) + 4
    
    h = h / 6.0  # Normalize to 0-1
    
    # Create mask for pixels matching source color
    # Match based on hue primarily, with some saturation consideration
    hue_diff = np.minimum(np.abs(h - src_h), 1 - np.abs(h - src_h))  # Circular distance
    hue_tolerance = tolerance / 360.0
    
    # For grayscale source colors (low saturation), match all hues
    if src_s < 0.1:
        mask = s < 0.1  # Match other grayscale pixels
    else:
        mask = (hue_diff < hue_tolerance) & (s > 0.1)
    
    # For matching pixels, change hue to target
    if target_s < 0.1:  # Target is grayscale
        s[mask] = 0
        h[mask] = 0
    else:
        h[mask] = target_h
        # Optionally adjust saturation towards target
        s[mask] = np.clip(s[mask] * (target_s / max(src_s, 0.1)), 0, 1)
    
    # Vectorized HSV to RGB conversion
    h_i = (h * 6.0).astype(int)
    f = h * 6.0 - h_i
    p = v * (1.0 - s)
    q = v * (1.0 - f * s)
    t = v * (1.0 - (1.0 - f) * s)
    
    h_i = h_i % 6
    
    # Initialize output arrays
    r_out = np.zeros_like(v)
    g_out = np.zeros_like(v)
    b_out = np.zeros_like(v)
    
    # Apply conversions based on hue segment
    mask0 = (h_i == 0)
    r_out[mask0] = v[mask0]
    g_out[mask0] = t[mask0]
    b_out[mask0] = p[mask0]
    
    mask1 = (h_i == 1)
    r_out[mask1] = q[mask1]
    g_out[mask1] = v[mask1]
    b_out[mask1] = p[mask1]
    
    mask2 = (h_i == 2)
    r_out[mask2] = p[mask2]
    g_out[mask2] = v[mask2]
    b_out[mask2] = t[mask2]
    
    mask3 = (h_i == 3)
    r_out[mask3] = p[mask3]
    g_out[mask3] = q[mask3]
    b_out[mask3] = v[mask3]
    
    mask4 = (h_i == 4)
    r_out[mask4] = t[mask4]
    g_out[mask4] = p[mask4]
    b_out[mask4] = v[mask4]
    
    mask5 = (h_i == 5)
    r_out[mask5] = v[mask5]
    g_out[mask5] = p[mask5]
    b_out[mask5] = q[mask5]
    
    # Update result array
    result[:, :, 0] = r_out * 255
    result[:, :, 1] = g_out * 255
    result[:, :, 2] = b_out * 255
    
    return result.astype(np.uint8)


def process_image(input_path, output_path, src_color, target_color, tolerance, show_compare=False):
    """Process a single image file."""
    # Load image
    try:
        img = Image.open(input_path)
        img = img.convert('RGBA')  # Ensure we have alpha channel
    except Exception as e:
        print(f"Error loading image {input_path}: {e}")
        return False
    
    # Convert to numpy array
    img_array = np.array(img)
    
    # Replace colors
    result_array = replace_color(img_array, src_color, target_color, tolerance)
    
    # Convert back to image
    result_img = Image.fromarray(result_array, 'RGBA')
    
    # Always save as PNG to preserve quality and support transparency
    output_path_str = str(output_path)
    if not output_path_str.lower().endswith('.png'):
        # Replace extension with .png
        output_path_str = str(Path(output_path_str).with_suffix('.png'))
    
    # Save output
    result_img.save(output_path_str)
    print(f"Processed: {input_path} -> {output_path}")
    
    # Show comparison if requested
    if show_compare:
        show_comparison(img, result_img)
    
    return True


def process_yolo_dataset(input_dir, output_dir, src_color, target_color, tolerance):
    """Process a YOLO format dataset, preserving structure and copying labels."""
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    
    # Check for data.yaml
    data_yaml = input_path / 'data.yaml'
    if not data_yaml.exists():
        print(f"Warning: data.yaml not found in {input_path}")
        print("Proceeding as a standard directory...")
        return False
    
    # Create output directory
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Copy data.yaml
    shutil.copy2(data_yaml, output_path / 'data.yaml')
    print(f"Copied: data.yaml")
    
    # Read data.yaml to get dataset structure
    with open(data_yaml, 'r') as f:
        data_config = yaml.safe_load(f)
    
    # Common YOLO dataset splits
    splits = ['train', 'val', 'test']
    total_images = 0
    processed_images = 0
    
    for split in splits:
        split_path = input_path / split
        if not split_path.exists():
            continue
        
        print(f"\nProcessing {split} split...")
        
        # Create output split directories
        output_split_images = output_path / split / 'images'
        output_split_labels = output_path / split / 'labels'
        output_split_images.mkdir(parents=True, exist_ok=True)
        output_split_labels.mkdir(parents=True, exist_ok=True)
        
        # Process images
        images_dir = split_path / 'images'
        labels_dir = split_path / 'labels'
        
        if images_dir.exists():
            image_files = list(images_dir.glob('*.jpg')) + list(images_dir.glob('*.png')) + \
                         list(images_dir.glob('*.jpeg')) + list(images_dir.glob('*.JPG')) + \
                         list(images_dir.glob('*.PNG'))
            
            total_images += len(image_files)
            
            for img_file in image_files:
                # Process image - keep the full original name, just change extension to .png
                output_img = output_split_images / (img_file.stem + '.png')
                
                if process_image(img_file, output_img, src_color, target_color, tolerance):
                    processed_images += 1
                
                # Copy corresponding label file if it exists
                # Label files should match the original image stem
                if labels_dir.exists():
                    label_file = labels_dir / f"{img_file.stem}.txt"
                    if label_file.exists():
                        output_label = output_split_labels / f"{img_file.stem}.txt"
                        shutil.copy2(label_file, output_label)
    
    # Copy any additional files in the root directory
    for item in input_path.iterdir():
        if item.is_file() and item.name != 'data.yaml':
            shutil.copy2(item, output_path / item.name)
            print(f"Copied: {item.name}")
    
    print(f"\n{'='*60}")
    print(f"YOLO Dataset Processing Complete!")
    print(f"Total images processed: {processed_images}/{total_images}")
    print(f"Output saved to: {output_path}")
    print(f"{'='*60}")
    
    return True


def main():
    parser = argparse.ArgumentParser(
        description='Replace shades of a source color with a target color in an image, directory, or YOLO dataset'
    )
    parser.add_argument('input', help='Input image file, directory, or YOLO dataset directory')
    parser.add_argument('--src', required=True, help='Source color (e.g., red, #FF0000, 255,0,0)')
    parser.add_argument('--target', required=True, help='Target color (e.g., yellow, #FFFF00, 255,255,0)')
    parser.add_argument('--output', '-o', help='Output image file, directory, or dataset directory')
    parser.add_argument('--tolerance', '-t', type=int, default=30, 
                       help='Color matching tolerance 0-100 (default: 30)')
    parser.add_argument('--compare', '-c', action='store_true',
                       help='Show original and transformed images side-by-side (only for single images)')
    parser.add_argument('--yolo', action='store_true',
                       help='Process as YOLO dataset (preserves structure and copies labels)')
    parser.add_argument('--extensions', nargs='+', 
                       default=['.png', '.jpg', '.jpeg', '.bmp', '.gif', '.tiff'],
                       help='Image file extensions to process in directory mode (default: .png .jpg .jpeg .bmp .gif .tiff)')
    
    args = parser.parse_args()
    
    # Parse colors
    try:
        src_color = parse_color(args.src)
        target_color = parse_color(args.target)
    except ValueError as e:
        print(f"Error: {e}")
        return 1
    
    print(f"Replacing {args.src} {src_color} with {args.target} {target_color}...")
    
    input_path = Path(args.input)
    
    # Check if input is a directory
    if input_path.is_dir():
        # Check if YOLO mode is requested or if it looks like a YOLO dataset
        if args.yolo or (input_path / 'data.yaml').exists():
            output_dir = Path(args.output) if args.output else Path(f'{input_path.name}_colorshifted')
            if process_yolo_dataset(input_path, output_dir, src_color, target_color, args.tolerance):
                return 0
            # If YOLO processing failed, fall through to regular directory mode
        
        # Regular directory mode
        output_dir = Path(args.output) if args.output else Path('output_dir')
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # Find all image files
        image_files = []
        for ext in args.extensions:
            image_files.extend(input_path.glob(f'*{ext}'))
            image_files.extend(input_path.glob(f'*{ext.upper()}'))
        
        if not image_files:
            print(f"No image files found in {input_path}")
            print(f"Looking for extensions: {args.extensions}")
            return 1
        
        print(f"Found {len(image_files)} image(s) to process")
        
        success_count = 0
        for img_file in image_files:
            output_file = output_dir / img_file.stem
            output_file = output_file.with_suffix('.png')
            if process_image(img_file, output_file, src_color, target_color, args.tolerance):
                success_count += 1
        
        print(f"\nCompleted: {success_count}/{len(image_files)} images processed successfully")
        print(f"Output saved to: {output_dir}")
        
    else:
        # Single file mode
        if not input_path.exists():
            print(f"Error: Input file not found: {input_path}")
            return 1
        
        output_path = args.output or 'output.png'
        
        if process_image(input_path, output_path, src_color, target_color, 
                        args.tolerance, args.compare):
            return 0
        else:
            return 1
    
    return 0


if __name__ == '__main__':
    exit(main())
