import cv2
import os
import sys

# Directory containing your images
image_dir = "/bos/src/yolo/test_images/" + sys.argv[1]
print("reading from dir: " + image_dir)

# Supported image extensions
valid_exts = (".jpg", ".jpeg", ".png", ".bmp", ".tiff")

# Get list of all valid image files in the directory
image_files = [f for f in os.listdir(image_dir) if f.lower().endswith(valid_exts)]
image_files.sort()  # optional: ensures a consistent order

if not image_files:
    print("No images found in", image_dir)
    exit(1)

for filename in image_files:
    path = os.path.join(image_dir, filename)
    print("Displaying:", path)
    
    # Read the image
    image = cv2.imread(path)
    if image is None:
        print("Failed to read", path)
        continue
    
    # Display image
    cv2.imshow("image", image)
    
    # Wait for any key press
    key = cv2.waitKey(0)
    
    # If user presses ESC, exit early
    if key == 27:
        print("Exiting early.")
        break

# Close all windows when done
cv2.destroyAllWindows()
