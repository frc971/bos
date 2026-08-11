import os
import sys
import re
from PIL import Image, ImageTk
import tkinter as tk

SUPPORTED_EXTENSIONS = (".png", ".jpg", ".jpeg", ".bmp", ".gif", ".tiff")


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

        # Collect images
        items = []
        for f in os.listdir(folder):
            if f.lower().endswith(SUPPORTED_EXTENSIONS):
                try:
                    ts = extract_timestamp(f)
                    items.append((ts, os.path.join(folder, f)))
                except ValueError:
                    pass

        if not items:
            print("No valid timestamped images found.")
            sys.exit(1)

        # Sort numerically
        items.sort(key=lambda x: x[0])

        # --- FILTER BY START TIME ---
        items = [item for item in items if item[0] >= start_time]

        if not items:
            print(f"No images found after start_time={start_time}")
            sys.exit(1)

        self.timestamps = [x[0] for x in items]
        self.image_paths = [x[1] for x in items]

        # --- NORMALIZE TO FIRST DISPLAYED FRAME ---
        self.t0 = self.timestamps[0]
        self.norm_timestamps = [t - self.t0 for t in self.timestamps]

        self.index = 0

        self.label = tk.Label(self.root)
        self.label.pack()

        self.root.bind("<Key>", self.next_image)

        self.show_image()
        self.root.mainloop()

    def show_image(self):
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

    def next_image(self, event=None):
        self.index += 1
        if self.index >= len(self.image_paths):
            print("Reached end of images.")
            self.root.quit()
            return

        self.show_image()


if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        print("Usage: python viewer.py <image_folder> [start_time]")
        sys.exit(1)

    folder = sys.argv[1]
    start_time = float(sys.argv[2]) if len(sys.argv) == 3 else 0.0

    ImageViewer(folder, start_time)
