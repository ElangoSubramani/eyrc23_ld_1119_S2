# import the necessary packages
from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2
import argparse

# Create ArgumentParser object
parser = argparse.ArgumentParser(description='Image detection script.')

# Add arguments
parser.add_argument('image_path', help='Path to the image file.')

# Parse the command line arguments
args = parser.parse_args()

# Access the value of the argument
image_path = args.image_path

# Use the argument value in your script
print(f'The path to the image is: {image_path}')


image = cv2.imread(image_path, 1)

# convert it to grayscale, and blur it7
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (11, 11), 0)

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

# Loop over the contours
# ...
# Loop over the contours
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
    # Adjust the divisor for border thickness
    cv2.drawContours(image, [c], -1, (0, 0, 255), int(perimeter/70))

# Save the output image as a PNG file
cv2.imwrite("led_detection_results.png", image)
# ...

# ...

# Open a text file for writing
with open("led_detection_results.txt", "w") as file:
    # Write the number of LEDs detected to the file
    file.write(f"No. of LEDs detected: {led_count}\n")

    # Loop over the contours
    for i, (centroid, area) in enumerate(zip(centroid_list, area_list)):
        # Write centroid coordinates and area for each LED to the file
        file.write(f"Centroid #{i + 1}: {centroid}\nArea #{i + 1}: {area}\n")
        file.write("\n")
