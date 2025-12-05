import cv2
import os
import sys

image_dir = "/bos/src/yolo/test_images/" + sys.argv[1]
print("reading from dir: " + image_dir)

# Actually cv seems to struggle with exporting to jpg, should mainly be bmp if you used the img_collector.cc file
valid_exts = (".jpg", ".jpeg", ".png", ".bmp", ".tiff")

image_files = [f for f in os.listdir(image_dir) if f.lower().endswith(valid_exts)]
image_files.sort()

if not image_files:
    print("No images found in", image_dir)
    exit(1)

for filename in image_files:
    path = os.path.join(image_dir, filename)
    print("Displaying:", path)
    
    image = cv2.imread(path)
    if image is None:
        print("Failed to read", path)
        continue
    
    cv2.imshow("image", image)
    
    key = cv2.waitKey(0)
    
    # If user presses ESC, exit early
    if key == 27:
        print("Exiting early.")
        break

cv2.destroyAllWindows()
