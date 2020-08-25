####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import rospy

from skimage.feature import canny
from skimage.io import imread
from skimage.transform import resize


class EllipseDetector(object):
    """Detects ellipses in an image and returns their locations."""

    def __init__(self):
        #rospy.loginfo("Creating the ellipse detector.")
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
        #rospy.loginfo("Running ellipse detection.")
        _, contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) >= 1:
            n = len(contours)
            # Get the two largest ellipses (i.e. the eyes, not any dirt)
            contours = sorted(contours, key=lambda c: c.shape[0], reverse=True)[:n]
            # Sort them that first ellipse is always the left eye (in the image)
            contours = sorted(contours, key=np.max)
    
            # Fit the ellipses for the two eyes
            e = []
            for i in contours:
                if len(i) > 4:
                    e.append(cv2.fitEllipse(i))
            return e
        else:
            # Not at least two eyes + maybe dirt found...
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
    #imgs = ['/home/pi/workspaces/steve/src/steve/steve_auv/test/steve_auv/data/IMG_2952.jpg']
    out_dir = '/home/pi'
    for img in imgs:
        image = imread(img, as_gray=True)
        image_final = imread(img, as_gray=False)
        sf = 8.0
        scale = (image.shape[0] / (image.shape[0] // sf), image.shape[1] / (image.shape[1] // sf))
        shape_origin = image.shape
        image = resize(image, (image.shape[0] // sf, image.shape[1] // sf), anti_aliasing=True)

        #from skimage.filters import threshold_otsu
        #thresh = threshold_otsu(image)
        #plt.imshow(image, cmap=plt.cm.gray)
        #plt.show()
        edges = canny(image, sigma=.5, low_threshold=0.60, high_threshold=0.80)
        #from scipy import ndimage as ndi
        #edges = ndi.binary_fill_holes(edges)
        #from skimage.morphology import dilation
        #edges = dilation(edges)
        #from skimage.filters import sobel
        #edges = sobel(image)
        #markers = np.zeros_like(image)
        #markers[image < 30] = 1
        #markers[image > 150] = 2
        #from skimage.segmentation import watershed
        #edges = watershed(edges, markers)
        #plt.imshow(edges, cmap=plt.cm.gray)
        #plt.show()
        edges = np.asarray(255 * edges, np.uint8)
        #print(np.min(edges))
        #print(np.max(edges))
        #edges = np.asarray(255 * edges, np.uint8)
        _, edges = cv2.threshold(edges, 127, 255, cv2.THRESH_BINARY)
        edges_final = resize(edges, shape_origin, anti_aliasing=True)
        #plt.imshow(edges, cmap=plt.cm.gray)
        #plt.show()

        detector = EllipseDetector()
        ellipses = detector.detect(edges)
        if ellipses != False:
            for i in ellipses:
                i = ((i[0][0] * scale[0], i[0][1] * scale[1]), (i[1][0] * scale[0], i[1][1] * scale[1]), i[2])
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
