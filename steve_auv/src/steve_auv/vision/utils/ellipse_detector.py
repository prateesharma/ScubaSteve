####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import cv2
import numpy as np
import rospy


class EllipseDetector(object):
    """Detects ellipses in an image and returns their locations."""

    def __init__(self):
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
        rospy.loginfo("Running ellipse detection.")
        keypoints = self._detector.detect(image)
        return keypoints


def main():
    image = cv2.imread(("/home/pi/workspaces/steve/src/steve/steve_auv/test/"
                        "steve_auv/data/coins.jpg"),
                       cv2.IMREAD_GRAYSCALE
                      )
    image_final = cv2.imread(("/home/pi/workspaces/steve/src/steve/steve_auv/test/"
                              "steve_auv/data/coins.jpg"),
                             cv2.IMREAD_COLOR
                            )
    print(image.shape)

    from skimage import color, img_as_ubyte
    from skimage.feature import canny
    from skimage_transform import hough_ellipse
    from skimage.draw import ellipse_perimeter

    edges = canny(image, sigma=2.0, low_threshold=0.55, high_threshold=0.8)
    result = hough_ellipse(edges, accuracy=20, threshold=250, mmin_size=100, max_size=120)
    result.sort(order='accumulator')

    #detector = EllipseDetector()

    #image = cv2.blur(image, (10, 10)) 
    #image = cv2.dilate(image, np.ones((10, 10), np.uint8), iterations=1)
    #image = cv2.blur(image, (10, 10))

    #_, image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY_INV)
    #image = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 2)
    #_, contours, hierarchy = cv2.findContours(image, 2, 1)
    #keep = []
    #for i in contours:
    #    area = cv2.contourArea(i)
    #    if area > 50000:
    #        keep.append(i)
    #print(len(keep))
    #keypoints = detector.detect(image)
    #image_out = cv2.drawKeypoints(
    #                image,
    #                keypoints,
    #                np.zeros((1, 1)),
    #                (0, 0, 255),
    #                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
    #            )
    #
    #image_final = cv2.drawContours(image_final, keep, -1, (0, 255, 0), 3)
    cv2.imshow("Ellipses", edges)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
