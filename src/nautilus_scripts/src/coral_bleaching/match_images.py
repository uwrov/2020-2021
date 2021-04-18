#!/usr/bin/env python3

#Macros:
#To adjust, keep in mind (0,0) is the top corner, +x is to right, and +y is to the bottom. 
LEFT_BOUND = 250
RIGHT_BOUND = 650
UPPER_BOUND = 00

#Feature Matching: 
NUM_FEATURES = 500
NUM_MATCHES = 90




#Image IO:
def read_image(image_file):
    img = cv.imread(image_file)
    return cv.cvtColor(img, cv.COLOR_BGR2RGB)

def show_image(image, title):
    fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(14, 14))
    ax.imshow(image, cmap='gray')
    ax.set_title(title)
    ax.axis('off')
    return fig, ax

#Preprocessing:
def grayscale(image):
    HSV = cv.cvtColor(image, cv.COLOR_RGB2HSV)
    return HSV[:,:,2]

def crop(image):
    xmax = image.shape[0]
    return image[UPPER_BOUND:xmax, LEFT_BOUND:RIGHT_BOUND]

#Feature Detection:
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

# requires rgb image
# get n dominant colors using k-means clustering
def dominant_colors(img, n_colors):
    pixels = np.float32(img.reshape(-1, 3))

    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 200, .1)
    flags = cv.KMEANS_RANDOM_CENTERS

    _, labels, palette = cv.kmeans(pixels, n_colors, None, criteria, 10, flags)
    _, counts = np.unique(labels, return_counts=True)

    return palette

#All-in-one alignment, simply pass in preprocessed images:
def align_images(source_img, dest_img) :
    #Get keypoints:
    source_points, dest_points, _ = match_features(*mark_features(source_img), *mark_features(dest_img))
    #Estimate Homography Transformation:
    homography, _ = cv.findHomography(source_points, dest_points, cv.RANSAC, 5.0)
    #Apply Homography Transformation:
    y, x = dest_img.shape[:2]
    transformed_img = cv.warpPerspective(source_img, homography, (x, y), borderValue=bg_color(source_img))  
    #^Warp perspective > perspective Transform
    return transformed_img

# Segmentation:
# otsu requires grayscale image
def subtract_background(image):
    processed = image.copy()
    background_threshold = filters.threshold_otsu(processed)
    processed[processed < background_threshold] = 0
    return processed

def segment(image, n=2):
    #OpenCV Boilerplate:
    buffer = np.float32(image.flatten())
    criteria = (cv.TERM_CRITERIA_EPS + cv.TermCriteria_MAX_ITER, 10, 1.0)
    ret, label, center = cv.kmeans(buffer, n, None, criteria, 10, cv.KMEANS_RANDOM_CENTERS)
    center = np.uint8(center)
    res = center[label.flatten()]
    return center, res.reshape(image.shape)

#All-in-one segmentation:
def segment3(image): 
    processed = image.copy() 
    #Remove background noise for clustering
    processed = subtract_background(processed)
    #Cluster
    center, processed = segment(processed, 3)
    #Map three cluster colors to high contrast values: 30, 100, 200
    center = np.sort(center.flatten())
    processed = np.where(processed==center[0], 30, processed) 
    processed = np.where(processed==center[1], 100, processed) 
    processed = np.where(processed==center[2], 200, processed)
    return processed

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
    
    gamma = 0.014130435 * maxAvgV - 2.5846
    gamma = min(gamma, 1) # if image is bright enough, gamma = 1 leaves image unchanged
    return gamma_correct(image, gamma)

# returns pink mask, white mask
def segment_hsv(image):
    image = auto_gamma(image)
    image = cv.cvtColor(image, cv.COLOR_RGB2HSV)

    # red
    lower = np.array([0, 70, 50])
    upper = np.array([10, 255, 255])
    mask1 = cv.inRange(image, lower, upper)
    lower = np.array([170, 70, 50])
    upper = np.array([180, 255, 255])
    mask2 = cv.inRange(image, lower, upper)
    # pink
    lower = np.array([150, 70, 50])
    upper = np.array([170, 255, 255])
    mask3 = cv.inRange(image, lower, upper)
    mask_pink = mask1 | mask2 | mask3

    # white
    lower = np.array([0, 0, 180])
    upper = np.array([180, 70, 255])
    mask_white = cv.inRange(image, lower, upper)
    
    return mask_pink, mask_white

def reduce_noise(image):
    kernel = np.ones((3, 3), np.uint8)
    erosion = cv.erode(image, kernel, iterations = 7)
    dilation = cv.dilate(erosion, kernel, iterations = 9)
    return dilation

def process_frame():
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

    oldCoral_rect = alignedLeft_color.copy()
    newCoral_rect = newCoral.copy()

    for pair in contours:
        x, y, w, h = cv.boundingRect(pair[0])
        box_color = box_colors[pair[1]]
        cv.rectangle(oldCoral_rect, (x,y), (x+w, y+h), color=box_color, thickness=2)
        cv.rectangle(newCoral_rect, (x,y), (x+w, y+h), color=box_color, thickness=2)

# flatten image to dominant colors test
# requires rgb image
def flatten(image, n=3):
    buffer = np.float32(image.reshape(-1, 3))
    criteria = (cv.TERM_CRITERIA_EPS + cv.TermCriteria_MAX_ITER, 200, 0.1)
    _, labels, centers = cv.kmeans(buffer, n, None, criteria, 10, cv.KMEANS_RANDOM_CENTERS)
    centers = np.uint8(centers)
    flattened_image = centers[labels.flatten()].reshape(image.shape)
    return labels, centers, flattened_image

def adjust_contrast(image, a):
    #Enforce range of [0, 255]
    contrast_map = np.arange(0.0, 256.0) * float(a)
    above_255_indicies = contrast_map > 255
    below_0_indicies = contrast_map < 0
    contrast_map[above_255_indicies] = 255
    contrast_map[below_0_indicies] = 0

    #Apply Transformation
    processed = image.copy()
    xrange, yrange = image.shape
    for x in range(xrange) :
        for y in range(yrange) :
            processed[x][y] = contrast_map[processed[x][y]]
    return processed

def adjust_brightness(image, b):
    #Enforce range of [0, 255]
    brightness_map = np.arange(0.0, 256.0) + float(b)
    above_255_indicies = brightness_map > 255
    below_0_indicies = brightness_map < 0
    brightness_map[above_255_indicies] = 255
    brightness_map[below_0_indicies] = 0

    #Apply Transformation
    processed = image.copy()
    xrange, yrange = image.shape
    for x in range(xrange) :
        for y in range(yrange) :
            processed[x][y] = brightness_map[processed[x][y]]
    return processed