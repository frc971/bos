import cv2
import glob

collected_img_dir="/bos/src/yolo/training/collected_imgs/"
cv2.namedWindow("Collected Frame", cv2.WINDOW_NORMAL)

for path in glob.glob(collected_img_dir + "*.bmp"):
    cv2.imshow("Collected Frame", cv2.imread(path))
    cv2.waitKey(0)

cv2.destroyAllWindows()
