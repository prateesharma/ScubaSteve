####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import cv2
import numpy as np
#import rospy


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
        keypoints = self._detector.detect(image)
        return keypoints


def main():
    #image = cv2.imread(("/home/pi/workspaces/steve/src/steve/steve_auv/test/"
    #                    "steve_auv/data/coins.jpg"),
    #                   cv2.IMREAD_GRAYSCALE
    #                  )
    #image_final = cv2.imread(("/home/pi/workspaces/steve/src/steve/steve_auv/test/"
    #                          "steve_auv/data/coins.jpg"),
    #                         cv2.IMREAD_COLOR
    #                        )
    import cv2
    import skimage
    from skimage import color, img_as_ubyte
    from skimage.feature import canny
    from skimage.transform import hough_ellipse, resize
    from skimage.draw import ellipse_perimeter
    import matplotlib.pyplot as plt
    import numpy as np

    image = skimage.io.imread(("/home/cloud-user/repos/scuba_steve/steve_auv/test/"
                               "steve_auv/data/coins.jpg"),
                               as_gray=True
                             )
    image_final = skimage.io.imread(("/home/cloud-user/repos/scuba_steve/steve_auv/test/"
                                     "steve_auv/data/coins.jpg"),
                                     as_gray=False
                                   )
    image = resize(image, (image.shape[0] // 8, image.shape[1] // 8), anti_aliasing=True)
    print(image.shape)
    image_final = resize(image_final, (image_final.shape[0] // 8, image_final.shape[1] // 8), anti_aliasing=True)
    #plt.imshow(image, cmap=plt.cm.gray)
    #plt.show()

    edges = canny(image, sigma=1.00, low_threshold=0.60, high_threshold=0.90)
    #plt.imshow(edges)
    #plt.show()

    #result = hough_ellipse(edges, threshold=250, accuracy=20, min_size=25)
    #result.sort(order='accumulator')

    # Estimated parameters for the ellipse
    #best = list(result[-1])
    #yc, xc, xa, xb = [int(round(x)) for x in best[1:5]]
    #orientation = best[5]

    # Draw the ellipse on the original image
    #cy, cx = ellipse_perimeter(yc, xc, xa, xb, orientation)
    #import pdb; pdb.set_trace()
    #image_final[cy, cx] = (0, 0, 255)

    # Draw the edge (white) and the resulting ellipse (red)
    #edges = color.gray2rgb(img_as_ubyte(edges))
    #edges[cy, cx] = (250, 0, 0)

    #def auto_canny(image, sigma=0.1):
    #    # compute the median of the single channel pixel intensities
    #    v = np.median(image)
    #    # apply automatic Canny edge detection using the computed median
    #    lower = int(max(0, (1.0 - sigma) * v))
    #    upper = int(min(255, (1.0 + sigma) * v))
    #    edged = cv2.Canny(image, lower, upper)
    #    # return the edged image
    #    return edged

    #image = cv2.imread(("/home/cloud-user/repos/scuba_steve/steve_auv/test/"
    #                    "steve_auv/data/coins.jpg")
    #                  )
    #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #gray = cv2.resize(gray, (gray.shape[1] // 8, gray.shape[0] // 8))
    #edges = auto_canny(gray)
    #plt.imshow(edges, cmap=plt.cm.gray)
    #plt.show()

    detector = EllipseDetector()
    edges = np.asarray(255 * edges, np.uint8)
    _, edges = cv2.threshold(edges, 127, 255, cv2.THRESH_BINARY)

    def findEllipses(edges):
        contours, _ = cv2.findContours(edges.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        ellipseMask = np.zeros(edges.shape, dtype=np.uint8)
        contourMask = np.zeros(edges.shape, dtype=np.uint8)

        pi_4 = np.pi * 4

        for i, contour in enumerate(contours):
            if len(contour) < 5:
                continue

            area = cv2.contourArea(contour)
            if area <= 100:  # skip ellipses smaller then 10x10
                continue

            arclen = cv2.arcLength(contour, True)
            circularity = (pi_4 * area) / (arclen * arclen)
            ellipse = cv2.fitEllipse(contour)
            poly = cv2.ellipse2Poly((int(ellipse[0][0]), int(ellipse[0][1])), (int(ellipse[1][0] / 2), int(ellipse[1][1] / 2)), int(ellipse[2]), 0, 360, 5)

            # if contour is circular enough
            if circularity > 0.6:
                cv2.fillPoly(ellipseMask, [poly], 255)
                continue

            # if contour has enough similarity to an ellipse
            similarity = cv2.matchShapes(poly.reshape((poly.shape[0], 1, poly.shape[1])), contour, cv2.cv.CV_CONTOURS_MATCH_I2, 0)
            if similarity <= 0.2:
                cv2.fillPoly(contourMask, [poly], 255)

        return ellipseMask, contourMask    

    def _fit_ellipse(thresholded_image):
        """Finds contours and fits an ellipse to thresholded image
    
        Parameters
        ----------
        thresholded_image :
            Binary image containing two eyes

        Returns
        -------
        type
            When eyes were found, the two ellipses, otherwise False

        """
        cont_ret = cv2.findContours(
            thresholded_image.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        # API change, in OpenCV 4 there are 2 values unlike OpenCV3
        if len(cont_ret) == 3:
            _, contours, hierarchy = cont_ret
        else:
            contours, hierarchy = cont_ret
    
        if len(contours) >= 2:
            n = len(contours)
            # Get the two largest ellipses (i.e. the eyes, not any dirt)
            contours = sorted(contours, key=lambda c: c.shape[0], reverse=True)[:n]
            # Sort them that first ellipse is always the left eye (in the image)
            contours = sorted(contours, key=np.max)
    
            # Fit the ellipses for the two eyes
            if len(contours[0]) > 4 and len(contours[1]) > 4:
                e = [cv2.fitEllipse(contours[i]) for i in range(n)]
                return e
            else:
                return False
    
        else:
            # Not at least two eyes + maybe dirt found...
            return False
    
    ellipses = _fit_ellipse(edges)
    import pdb; pdb.set_trace()
    #contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #keep = []
    #for i in contours:
    #    area = cv2.contourArea(i)
    #    if area > 100:
    #        keep.append(i)
    #keypoints = detector.detect(edges)
    #image_out = cv2.drawKeypoints(
    #                edges,
    #                keypoints,
    #                np.zeros((1, 1)),
    #                (0, 0, 255),
    #                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
    #            )
    import copy 
    image_final_out = copy.deepcopy(image_final) 
    #image_final_out = cv2.drawContours(image_final_out, keep, -1, (0, 0, 255), 3)
    print(len(ellipses))
    for i in ellipses:
        cv2.ellipse(image_final_out, i, (0, 0, 255))
    fig2, (ax1, ax2) = plt.subplots(ncols=2, nrows=1, figsize=(8, 4),
                                    sharex=True, sharey=True)
    ax1.set_title('Original picture')
    ax1.imshow(image_final)
    ax2.set_title('Edge (white) and result (red)')
    ax2.imshow(image_final_out)
    plt.show()

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
    #cv2.imshow("Ellipses", edges)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    #plt.imshow(edges, cmap=plt.cm.gray)
    #plt.show()


if __name__ == "__main__":
    main()
