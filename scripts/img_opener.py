import os
import sys
import re
from bisect import bisect_left
from PIL import Image, ImageTk
import tkinter as tk

SUPPORTED_EXTENSIONS = (".png", ".jpg", ".jpeg", ".bmp", ".gif", ".tiff")
CAMERA_DIRS = ("main_bot_left", "main_bot_right")


def extract_timestamp(filename):
    match = re.search(r"\d+(\.\d+)?", filename)
    if match:
        return float(match.group())
    else:
        raise ValueError(f"No timestamp found in filename: {filename}")


class ImageViewer:
    def __init__(self, folder, start_time=0.0):
        self.root = tk.Tk()
        self.root.title("Image Viewer (Press any key to advance)")

        folder = resolve_folder(folder)
        camera_folders = [os.path.join(folder, cam) for cam in CAMERA_DIRS]
        if all(os.path.isdir(camera_folder) for camera_folder in camera_folders):
            self.load_stereo_images(camera_folders, start_time)
        else:
            self.load_single_folder(folder, start_time)

        self.index = 0

        self.label = tk.Label(self.root)
        self.label.pack()

        self.root.bind("<Key>", self.next_image)

        self.show_image()
        self.root.mainloop()

    def load_single_folder(self, folder, start_time):
        self.mode = "single"
        items = collect_images(folder)

        if not items:
            print("No valid timestamped images found.")
            sys.exit(1)

        items = [item for item in items if item[0] >= start_time]

        if not items:
            print(f"No images found after start_time={start_time}")
            sys.exit(1)

        self.timestamps = [x[0] for x in items]
        self.image_paths = [x[1] for x in items]
        self.t0 = self.timestamps[0]
        self.norm_timestamps = [t - self.t0 for t in self.timestamps]

    def load_stereo_images(self, camera_folders, start_time):
        self.mode = "stereo"
        left_items = collect_images(camera_folders[0])
        right_items = collect_images(camera_folders[1])

        if not left_items or not right_items:
            print("Both main_bot_left and main_bot_right need valid timestamped images.")
            sys.exit(1)

        left_items = [item for item in left_items if item[0] >= start_time]
        right_items = [item for item in right_items if item[0] >= start_time]

        if not left_items or not right_items:
            print(f"No images found after start_time={start_time}")
            sys.exit(1)

        all_timestamps = sorted(set(t for t, _ in left_items + right_items))

        left_timestamps = [x[0] for x in left_items]
        right_timestamps = [x[0] for x in right_items]
        pairs = []
        last_paths = None
        for timestamp in all_timestamps:
            left = nearest_item(left_items, left_timestamps, timestamp)
            right = nearest_item(right_items, right_timestamps, timestamp)
            paths = (left[1], right[1])
            if paths == last_paths:
                continue
            pairs.append((timestamp, left, right))
            last_paths = paths

        self.timestamps = [x[0] for x in pairs]
        self.image_pairs = [(x[1], x[2]) for x in pairs]
        self.t0 = self.timestamps[0]
        self.norm_timestamps = [t - self.t0 for t in self.timestamps]

    def show_image(self):
        if self.mode == "stereo":
            self.show_stereo_image()
        else:
            self.show_single_image()

    def show_single_image(self):
        path = self.image_paths[self.index]
        img = Image.open(path)

        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        img.thumbnail((screen_w, screen_h))

        self.tk_image = ImageTk.PhotoImage(img)
        self.label.config(image=self.tk_image)

        norm_time = self.norm_timestamps[self.index]

        self.root.title(
            f"{os.path.basename(path)} | t = {norm_time:.3f}s "
            f"({self.index+1}/{len(self.image_paths)})"
        )

    def show_stereo_image(self):
        (left_ts, left_path), (right_ts, right_path) = self.image_pairs[self.index]

        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        img = make_side_by_side(left_path, right_path, screen_w, screen_h)

        self.tk_image = ImageTk.PhotoImage(img)
        self.label.config(image=self.tk_image)

        norm_time = self.norm_timestamps[self.index]
        frame_time = self.timestamps[self.index]
        delta_ms = abs(left_ts - right_ts) * 1000
        self.root.title(
            f"left {os.path.basename(left_path)} | right {os.path.basename(right_path)} | "
            f"t = {norm_time:.3f}s ({frame_time:.3f}) | dt = {delta_ms:.1f}ms "
            f"({self.index+1}/{len(self.image_pairs)})"
        )

    def next_image(self, event=None):
        self.index += 1
        if self.index >= len(self.timestamps):
            print("Reached end of images.")
            self.root.quit()
            return

        self.show_image()


def resolve_folder(folder):
    if os.path.isdir(folder):
        return folder

    scripts_parent_path = os.path.join(os.path.dirname(__file__), "..", folder)
    if os.path.isdir(scripts_parent_path):
        return scripts_parent_path

    print(f"Folder not found: {folder}")
    sys.exit(1)


def collect_images(folder):
    items = []
    for f in os.listdir(folder):
        if f.lower().endswith(SUPPORTED_EXTENSIONS):
            try:
                ts = extract_timestamp(f)
                items.append((ts, os.path.join(folder, f)))
            except ValueError:
                pass

    items.sort(key=lambda x: x[0])
    return items


def nearest_item(items, timestamps, timestamp):
    index = bisect_left(timestamps, timestamp)
    if index == 0:
        return items[0]
    if index == len(items):
        return items[-1]

    before = items[index - 1]
    after = items[index]
    if abs(before[0] - timestamp) <= abs(after[0] - timestamp):
        return before
    return after


def make_side_by_side(left_path, right_path, screen_w, screen_h):
    max_each_w = max(1, screen_w // 2)
    left_img = Image.open(left_path).convert("RGB")
    right_img = Image.open(right_path).convert("RGB")
    left_img.thumbnail((max_each_w, screen_h))
    right_img.thumbnail((max_each_w, screen_h))

    width = left_img.width + right_img.width
    height = max(left_img.height, right_img.height)
    combined = Image.new("RGB", (width, height), "black")
    combined.paste(left_img, (0, (height - left_img.height) // 2))
    combined.paste(right_img, (left_img.width, (height - right_img.height) // 2))
    return combined


if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        print("Usage: python img_opener.py <image_folder> [start_time]")
        sys.exit(1)

    folder = sys.argv[1]
    start_time = float(sys.argv[2]) if len(sys.argv) == 3 else 0.0

    ImageViewer(folder, start_time)
