#!/usr/bin/env python3

'''
# Team ID:          ld_1119
# Theme:            Luminosity Drone
# Authors List:     Elango S, Hemasri m
# Filename:         LD_1119_led_detection.py
# Global variables: parser, args, image_path, image, gray, blurred, thresh, labels, mask, cnts, led_count, centroid_list, area_list, led_centroids, threshold_distance, linkage_matrix, labels, clusters
'''


"""
    This python file runs a image processing algorithm to detect the LEDs in the image.

    The algorithm is based on the following steps:

    1. Convert the image to grayscale and blur it slightly.
    2. Threshold the image to reveal light regions in the blurred image.
    3. Perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image.    
    4. Perform a connected component analysis on the thresholded image, then initialize a mask to store only the "large" components.
    5. Loop over the unique components.
    6. If the number of pixels in the component is sufficiently large, then add it to our mask of "large blobs".
    7. Find the contours in the mask, then sort them from left to right.
    8. Loop over the contours.
    9. Calculate the area of the contour.
    10. Calculate the centroid.
    11. Increment the LED count and draw the LED number as "+1", "+2", and so on.
    12. Calculate the contour perimeter for more accurate border size.
    13. Draw the red border with the calculated perimeter.
    14. Save the output image as a PNG file.
    15. Define the distance threshold.
    16. Perform hierarchical clustering.
    17. Assign clusters based on the distance threshold.
    18. Open a text file for writing.
    19. Write Organism Type, centroid coordinates and area for each LED to the file.
    22. Close the file.


"""


# import the necessary packages


# image processing imports
# Command line arugment imports
# Clustering imports
# from icecream import ic

# Create ArgumentParser object
from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2
import argparse
from scipy.cluster.hierarchy import linkage, fcluster, dendrogram
from collections import Counter
import numpy as np
parser = argparse.ArgumentParser(description='Image detection script.')

# Add arguments
parser.add_argument('--image', help='Path to the image file.')

# Parse the command line arguments
args = parser.parse_args()

# Access the value of the argument
image_path = args.image
# # Use the argument value in your script
# print(f'The path to the image is: {image_path}')


image = cv2.imread(image_path, 1)

# convert it to grayscale, and blur it
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# 11,11 converted to 5,5 to blur less
blurred = cv2.GaussianBlur(gray, (5, 5), 0)

# threshold the image to reveal light regions in the blurred image
thresh = cv2.threshold(blurred, 215, 255, cv2.THRESH_BINARY)[1]

# perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image
thresh = cv2.erode(thresh, None, iterations=2)
thresh = cv2.dilate(thresh, None, iterations=4)

# perform a connected component analysis on the thresholded image, then initialize a mask to store only the "large" components
labels = measure.label(thresh, connectivity=2, background=0)
mask = np.zeros(thresh.shape, dtype="uint8")

# loop over the unique components
for label in np.unique(labels):
    # if this is the background label, ignore it
    if label == 0:
        continue

    # otherwise, construct the label mask and count the number of pixels
    labelMask = np.zeros(thresh.shape, dtype="uint8")
    labelMask[labels == label] = 255
    numPixels = cv2.countNonZero(labelMask)

    # if the number of pixels in the component is sufficiently large, then add it to our mask of "large blobs"
    if numPixels > 0:
        mask = cv2.add(mask, labelMask)

# find the contours in the mask, then sort them from left to right
cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)
cnts = contours.sort_contours(cnts)[0]

# Initialize lists to store centroid coordinates and area as floating-point values
centroid_list = []
area_list = []


led_count = 0
for i, c in enumerate(cnts):
    # Calculate the area of the contour
    area = cv2.contourArea(c)

    # Calculate the centroid
    M = cv2.moments(c)
    if M["m00"] != 0:
        cX = float(M["m10"] / M["m00"])
        cY = float(M["m01"] / M["m00"])
        centroid_list.append((cX, cY))
        area_list.append(float(area))  # Convert area to float

        # Increment the LED count and draw the LED number as "+1", "+2", and so on
        led_count += 1
        label_text = "+" + str(led_count)
        cv2.putText(image, label_text, (int(cX) - 20, int(cY) - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # Calculate the contour perimeter for more accurate border size
    perimeter = cv2.arcLength(c, True)

    # Draw the red border with the calculated perimeter

    cv2.drawContours(image, [c], -1, (0, 0, 255), int(perimeter/70))

# Save the output image as a PNG file
cv2.imwrite("led_detection_results.png", image)
# ...

# //////////////////////////////////////////////////
# LED centeroid ariable used for clustering
led_centroids = np.array(centroid_list)
# Define the distance threshold
threshold_distance = 300.0

# Perform hierarchical clustering
# complete #ward both compatable for cluster merging wird is used for irregular shaped clusters and complete is used for spherical based cluster
linkage_matrix = linkage(led_centroids, method='ward', metric='euclidean')

# Assign clusters based on the distance threshold
labels = fcluster(linkage_matrix, t=threshold_distance, criterion='distance')
clusters = {f"cluster_{i}": (
    led_centroids[labels == i]).tolist() for i in np.unique(labels)}
# print(clusters)


for name, value in clusters.items():
    clusters[name] = {
        "values": value,
        "centroid": np.mean(value, axis=0).tolist(),
        "count": len(value)
    }
    if clusters[name]["count"] == 2:
        clusters[name]["type"] = "alien_a"

    elif clusters[name]["count"] == 3:
        clusters[name]["type"] = "alien_b"
    elif clusters[name]["count"] == 4:
        clusters[name]["type"] = "alien_c"
    elif clusters[name]["count"] == 5:
        clusters[name]["type"] = "alien_d"

# for name,value in clusters.items():
#     ic(name,value)
# print(clusters)
# ...

# Open a text file for writing
with open(f"{image_path[0:(len(image_path)-4):]}.txt", "w") as file:
    # Write the number of LEDs detected to the file
    # file.write(f"No. of LEDs detected: {led_count}\n")

    # Loop over the contours
    # for i, (centroid, area) in enumerate(zip(centroid_list, area_list)):
    #     # Write centroid coordinates and area for each LED to the file
    #     file.write(f"Centroid #{i + 1}: {centroid}\nArea #{i + 1}: {area}\n")
    #     file.write("\n")
    for name, value in clusters.items():
        type = clusters[name]["type"]
        centroid = clusters[name]["centroid"]
        file.write(f"Organism Type: {type}\n")
        file.write(f"Centroid: {centroid}\n")
        file.write("\n")
    # Close the file
    file.close()
