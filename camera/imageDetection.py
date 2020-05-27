
import cv2
import matplotlib.pyplot as plt
import cvlib as cv
from cvlib.object_detection import draw_bbox

#takes an image filepath
def detectImage(image1):
    im = cv2.imread(image1)
    cv.object_detection.populate_class_labels()
    bbox, label, conf = cv.detect_common_objects(im, confidence=0.25, model='yolov3-tiny')
    output_image = draw_bbox(im, bbox, label, conf)
    plt.imshow(output_image)
    plt.show()

    #Saying anything above 50% confidence is a POI
    if conf > .5:
        return True


if __name__ == '__main__':
    detectImage('image.jpg')
