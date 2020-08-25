####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
try:
    import rospy
except:
    pass
import sys

from skimage.feature import canny
from skimage.io import imread
from skimage.transform import resize


class EllipseDetector(object):
    """Detects ellipses in an image and returns their locations."""

    def __init__(self):
        if 'rospy' in sys.modules:
            rospy.loginfo("Creating the ellipse detector.")
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 10000
        params.maxArea = 500000
        params.filterByCircularity = False
        params.minCircularity = 0.5
        params.filterByConvexity = False
        params.minConvexity = 0.2
        params.filterByInertia = False
        params.minInertiaRatio = 0.01
        self._detector = cv2.SimpleBlobDetector_create(params)

    def detect(self, image):
        if 'rospy' in sys.modules:
            rospy.loginfo("Running ellipse detection.")
        _, contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) >= 1:
            n = len(contours)
            contours = sorted(contours, key=lambda c: c.shape[0], reverse=True)[:n]
            contours = sorted(contours, key=np.max)
            e = []
            for i in contours:
                if len(i) > 4:
                    e.append(cv2.fitEllipse(i)) # Need 5 points to fit ellipse
            return e
        else:
            return False


def main():
    imgs = [
            '/home/pi/workspaces/steve/src/steve/steve_auv/test/steve_auv/data/coins.jpg',
            '/home/pi/workspaces/steve/src/steve/steve_auv/test/steve_auv/data/IMG_2951.jpg',
            '/home/pi/workspaces/steve/src/steve/steve_auv/test/steve_auv/data/IMG_2952.jpg',
            '/home/pi/workspaces/steve/src/steve/steve_auv/test/steve_auv/data/IMG_2953.jpg',
            '/home/pi/workspaces/steve/src/steve/steve_auv/test/steve_auv/data/IMG_2954.jpg',
            '/home/pi/workspaces/steve/src/steve/steve_auv/test/steve_auv/data/IMG_2955.jpg',
            '/home/pi/workspaces/steve/src/steve/steve_auv/test/steve_auv/data/IMG_2956.jpg',
            '/home/pi/workspaces/steve/src/steve/steve_auv/test/steve_auv/data/IMG_2958.jpg',
            '/home/pi/workspaces/steve/src/steve/steve_auv/test/steve_auv/data/IMG_2959.jpg',
            '/home/pi/workspaces/steve/src/steve/steve_auv/test/steve_auv/data/IMG_2960.jpg',
            '/home/pi/workspaces/steve/src/steve/steve_auv/test/steve_auv/data/IMG_2961.jpg'
           ]
    out_dir = '/home/pi'
    for img in imgs:
        # Read in and downsample images
        image = imread(img, as_gray=True)
        image_final = imread(img, as_gray=False)
        sf = 8.0
        scale = (image.shape[0] / (image.shape[0] // sf), image.shape[1] / (image.shape[1] // sf))
        shape_origin = image.shape
        image = resize(image, (image.shape[0] // sf, image.shape[1] // sf), anti_aliasing=True)

        # Detect edges and trheshold
        edges = canny(image, sigma=.5, low_threshold=0.60, high_threshold=0.80)
        edges = np.asarray(255 * edges, np.uint8)
        _, edges = cv2.threshold(edges, 127, 255, cv2.THRESH_BINARY)
        edges_final = resize(edges, shape_origin, anti_aliasing=True)

        # Detect contours and fit ellipses
        detector = EllipseDetector()
        ellipses = detector.detect(edges)
        if ellipses != False:
            for i in ellipses:
                i = ((i[0][0] * scale[0], i[0][1] * scale[1]),
                     (i[1][0] * scale[0], i[1][1] * scale[1]),
                     i[2]
                    )
                cv2.ellipse(image_final, i, (255, 0, 0), 4)
            fig, (ax1, ax2) = plt.subplots(ncols=2, nrows=1, figsize=(8, 4), sharex=True, sharey=True)
            ax1.set_title('Canny Edge Detection')
            ax1.imshow(edges_final)
            ax2.set_title('Fitted Ellipses')
            ax2.imshow(image_final)
            filename = os.path.splitext(os.path.split(img)[1])[0]
            plt.savefig(os.path.join(out_dir, filename + "_ellipses.jpg"))


if __name__ == "__main__":
    main()
