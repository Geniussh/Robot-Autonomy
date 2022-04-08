import cv2
import numpy as np


def find_drink(rgb_image, lower, upper):
    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    _, cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    c = max(cnts, key = cv2.contourArea)
    contour = np.array(c).squeeze()
    center_x, center_y = np.mean(contour, axis=0).astype(int)
    print(center_x, center_y)
    cv2.circle(rgb_image, (center_x, center_y), radius=1, color=(0,255,0), thickness=10)
    cv2.drawContours(rgb_image, [c], 0, (0,255,0), 2)
    cv2.imwrite('green.png', rgb_image)

if __name__ == '__main__':
    rgb_image = cv2.imread('rgb.png')
    rLower = np.array([160,59,20])
    rUpper = np.array([179,255,255])

    gLower = np.array([70,69,20])
    gUpper = np.array([83,255,255])

    find_drink(rgb_image, gLower, gUpper)


# # Load image, grayscale, Otsu's threshold, and extract ROI
# image = cv2.imread('1.jpg')
# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
# x,y,w,h = cv2.boundingRect(thresh)
# ROI = image[y:y+h, x:x+w]

# # Color segmentation on ROI
# hsv = cv2.cvtColor(ROI, cv2.COLOR_BGR2HSV)
# lower = np.array([0, 0, 152])
# upper = np.array([179, 255, 255])
# mask = cv2.inRange(hsv, lower, upper)

# # Crop left and right half of mask
# x, y, w, h = 0, 0, ROI.shape[1]//2, ROI.shape[0]
# left = mask[y:y+h, x:x+w]
# right = mask[y:y+h, x+w:x+w+w]

# # Count pixels
# left_pixels = cv2.countNonZero(left)
# right_pixels = cv2.countNonZero(right)

# print('Left pixels:', left_pixels)
# print('Right pixels:', right_pixels)

# cv2.imshow('mask', mask)
# cv2.imshow('thresh', thresh)
# cv2.imshow('ROI', ROI)
# cv2.imshow('left', left)
# cv2.imshow('right', right)
# cv2.waitKey()