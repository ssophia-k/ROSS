import cv2
import numpy as np
from matplotlib import pyplot as plt
import os

ORB = cv2.ORB_create(nfeatures=10000)

img_path = os.path.join('sanjay-test', "images", 'test.jpg')
img2_path = os.path.join('sanjay-test', "images", 'test2.jpg')

img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
img2 = cv2.imread(img2_path, cv2.IMREAD_GRAYSCALE)
print(img.shape)
print(img2.shape)

keypt1, des1 = ORB.detectAndCompute(img, None)
keypt2, des2 = ORB.detectAndCompute(img2, None)

bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(des1, des2)
matches = sorted(matches, key=lambda x: x.distance)
output_img = cv2.drawMatches(img, keypt1, img2, keypt2, matches[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
plt.imshow(output_img)
plt.show()
