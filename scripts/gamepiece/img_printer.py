import cv2
import sys

img = cv2.imread(sys.argv[1])
cv2.imshow('Y', img)
cv2.waitKey(0);
cv2.destroyAllWindows()
