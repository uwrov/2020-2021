import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

# Macros:
# To adjust, keep in mind (0,0) is the top corner, +x is to right, and +y is to the bottom. 
LEFT_BOUND = 250
RIGHT_BOUND = 650
UPPER_BOUND = 00

# Feature Matching: 
NUM_FEATURES = 500
NUM_MATCHES = 90

# hsv color spaces for segmentation
# ranges are (lower, upper)
RED_RANGE_1 = (np.array([0, 70, 50]), np.array([10, 255, 255]))
RED_RANGE_2 = (np.array([170, 70, 50]), np.array([180, 255, 255]))
PINK_RANGE = (np.array([150, 70, 50]), np.array([170, 255, 255]))
WHITE_RANGE = (np.array([0, 0, 180]), np.array([180, 70, 255]))

def read_image(image_file):
    img = cv.imread(image_file)
    return cv.cvtColor(img, cv.COLOR_BGR2RGB)

def grayscale(image):
    HSV = cv.cvtColor(image, cv.COLOR_RGB2HSV)
    return HSV[:,:,2]

# Feature Detection:
def mark_features(image):
    image2 = image.copy()

    #ORB feature detection
    orb = cv.ORB_create(NUM_FEATURES)
    kp, des = orb.detectAndCompute(image2,None)
    for marker in kp:
	    image2 = cv.drawMarker(image2, tuple(int(i) for i in marker.pt), color=(0, 255, 0))
    return kp, des, image2

def match_features(kp1, des1, image1, kp2, des2, image2):
    #^keypoints, descriptors, and marked image respectively. Outputs of mark_features()
    #Feature Matching Algorithm, finds matches and selects the top ones:
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck = True)
    matches = bf.match(des1, des2)
    matches = sorted(matches, key = lambda x:x.distance)
    best_matches = matches[:NUM_MATCHES]

    #Get the points corresponding to the matches:
    p1 = []
    p2 = []
    for match in matches:
        p1.append(kp1[match.queryIdx].pt)   #points are stored in kp list. # old_kp
        p2.append(kp2[match.trainIdx].pt) #new_kp
    p1 = np.array(p1)
    p2 = np.array(p2)

    img3 = cv.drawMatches(image1, kp1, image2, kp2, best_matches, None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    return p1, p2, img3

def bg_color(img):
    # average color of row 0
    top_row = np.array([img[1]])
    average = np.mean(top_row, axis=(0,1))
    return average

# corrects color contrast/brightness
def gamma_correct(image, gamma):
    lookUpTable = np.empty((1,256), np.uint8)
    for i in range(256):
        lookUpTable[0,i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)
    return cv.LUT(image, lookUpTable)

# automatically calculates gamma based on average brightness of 10 x 10 kernel
# returns gamma-corrected (brightened) image
def auto_gamma(image):
    grayscale_image = grayscale(image)
    
    # get max average value (brightness) for 10 x 10 kernel
    maxAvgV = 0
    for x in range(0, grayscale_image.shape[0], 10):
        for y in range(0, grayscale_image.shape[1], 10):
            vSum = 0
            if (x + 10 < grayscale_image.shape[0] and y + 10 < grayscale_image.shape[1]):
                for i in range(10):
                    for j in range(10):
                        vSum += grayscale_image[x + i, y + j]
                if (vSum > maxAvgV):
                    maxAvgV = vSum
    maxAvgV /= 100
    
    gamma = 0.04 * pow(10, -7) * pow(1.07819, maxAvgV) + 0.30139
    gamma = min(gamma, 1) # if image is bright enough, gamma = 1 leaves image unchanged
    return gamma_correct(image, gamma)

# returns pink mask, white mask
def segment_hsv(image):
    image = auto_gamma(image)
    image = cv.cvtColor(image, cv.COLOR_RGB2HSV)

    # red
    mask1 = cv.inRange(image, RED_RANGE_1[0], RED_RANGE_1[1])
    mask2 = cv.inRange(image, RED_RANGE_2[0], RED_RANGE_2[1])
    # pink
    mask3 = cv.inRange(image, PINK_RANGE[0], PINK_RANGE[1])
    mask_pink = mask1 | mask2 | mask3

    # white
    mask_white = cv.inRange(image, WHITE_RANGE[0], WHITE_RANGE[1])
    
    return mask_pink, mask_white

def reduce_noise(image):
    kernel = np.ones((3, 3), np.uint8)
    erosion = cv.erode(image, kernel, iterations = 7)
    dilation = cv.dilate(erosion, kernel, iterations = 9)
    return dilation

# all oldCoral, newCoral images and masks should be aligned for subtraction to work
def bounding_boxes(oldCoral, oldCoral_mask, oldCoral_mask_pink, oldCoral_mask_white, newCoral, newCoral_mask, newCoral_mask_pink, newCoral_mask_white):
    # Mask subtraction
    """
    0 is bg
    Cases:
    - Growth: 0 -> pink, green
    - Damage: pink -> 0 or white -> 0, yellow
    - Bleaching: pink -> white, red
    - Recovery: white -> pink, blue
    """

    growth_sub = newCoral_mask_pink & ~oldCoral_mask
    damage_sub = oldCoral_mask & ~newCoral_mask
    bleach_sub = oldCoral_mask_pink & newCoral_mask_white
    recovery_sub = oldCoral_mask_white & newCoral_mask_pink

    # reduce noise:
    growth = reduce_noise(growth_sub)
    damage = reduce_noise(damage_sub)
    bleach = reduce_noise(bleach_sub)
    recovery = reduce_noise(recovery_sub)

    growth_contours, _ = cv.findContours(growth, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    damage_contours, _ = cv.findContours(damage, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    bleach_contours, _ = cv.findContours(bleach, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    recovery_contours, _ = cv.findContours(recovery, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    contours = [[c, 'g'] for c in growth_contours]
    contours += [[c, 'd'] for c in damage_contours]
    contours += [[c, 'b'] for c in bleach_contours]
    contours += [[c, 'r'] for c in recovery_contours]

    # top <= 4 contours:
    contours.sort(key=lambda pair: cv.contourArea(pair[0]), reverse=True)
    contours = contours[:4]

    # filter out duds:
    contours = [pair for pair in contours if cv.contourArea(pair[0]) > 1000] # ~30x30 pixels

    # draw bounding boxes on new and old aligned to new images:
    box_colors = {
        'g': (0, 255, 0), # growth: green
        'd': (255, 255, 0), # damage: yellow
        'b': (255, 0, 0), # bleaching: red
        'r': (0, 0, 255) # recovery: blue
    }

    oldCoral_rect = oldCoral.copy()
    newCoral_rect = newCoral.copy()

    # each pair = [contour, box color]
    for pair in contours:
        x, y, w, h = cv.boundingRect(pair[0])
        box_color = box_colors[pair[1]]
        cv.rectangle(oldCoral_rect, (x,y), (x+w, y+h), color=box_color, thickness=2)
        cv.rectangle(newCoral_rect, (x,y), (x+w, y+h), color=box_color, thickness=2)
    
    return oldCoral_rect, newCoral_rect

# oldCoral: cv image
# newCoral: cv image
# returns old coral, new coral images with colored bounding boxes around changes from the old to new images
def run_task(oldCoral, newCoral):
    oldCoral = cv.cvtColor(oldCoral, cv.COLOR_BGR2RGB)
    newCoral = cv.cvtColor(newCoral, cv.COLOR_BGR2RGB)

    # HSV hard-coded segmentation
    oldCoral_mask_pink, oldCoral_mask_white = segment_hsv(oldCoral)
    newCoral_mask_pink, newCoral_mask_white = segment_hsv(newCoral)

    oldCoral_mask = oldCoral_mask_pink | oldCoral_mask_white
    newCoral_mask = newCoral_mask_pink | newCoral_mask_white

    # alignment
    # get keypoints:
    src_pts, dst_pts, _ = match_features(*mark_features(oldCoral_mask), *mark_features(newCoral_mask))
    # estimate homography transformation using mask keypoints
    homography, _ = cv.findHomography(src_pts, dst_pts, cv.RANSAC, 5.0)

    # apply homography transformation to masks
    y, x = newCoral_mask.shape[:2]
    oldCoral_mask_aligned = cv.warpPerspective(oldCoral_mask, homography, (x, y), borderValue=(0,0,0))
    oldCoral_mask_pink_aligned = cv.warpPerspective(oldCoral_mask_pink, homography, (x, y), borderValue=(0,0,0))
    oldCoral_mask_white_aligned = cv.warpPerspective(oldCoral_mask_white, homography, (x, y), borderValue=(0,0,0))

    # apply homography transformation to original image
    y, x = newCoral.shape[:2]
    # perspective transform
    oldCoral_aligned = cv.warpPerspective(oldCoral, homography, (x, y), borderValue=bg_color(oldCoral))

    oldCoral_mask = oldCoral_mask_aligned
    oldCoral_mask_pink = oldCoral_mask_pink_aligned
    oldCoral_mask_white = oldCoral_mask_white_aligned
    
    oldCoral_rect, newCoral_rect = bounding_boxes(oldCoral_aligned, oldCoral_mask, oldCoral_mask_pink, oldCoral_mask_white, newCoral, newCoral_mask, newCoral_mask_pink, newCoral_mask_white)
    return cv.cvtColor(oldCoral_rect, cv.COLOR_RGB2BGR), cv.cvtColor(newCoral_rect, cv.COLOR_RGB2BGR)

# oldCoral_rect, newCoral_rect = run_task("colony_original_wall.JPG", "colony_dead_bleach_wall.JPG")

# # show images using OpenCV
# cv.imshow("old coral aligned to new coral", cv.cvtColor(oldCoral_rect, cv.COLOR_RGB2BGR))
# cv.imshow("new coral", cv.cvtColor(newCoral_rect, cv.COLOR_RGB2BGR))
# cv.waitKey(0)

# show images using matplotlib

# fig, ax = plt.subplots(1, 2, figsize=(12, 20))
# _ = ax[0].imshow(oldCoral_rect)
# _ = ax[0].set_title("old coral aligned to new coral")
# _ = ax[1].imshow(newCoral_rect)
# _ = ax[1].set_title("new coral")
# plt.show()
