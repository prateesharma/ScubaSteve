# from imageai.Detection import ObjectDetection
# import os
#
# execution_path = os.getcwd()
#
# detector = ObjectDetection()
# detector.setModelTypeAsRetinaNet()
# detector.setModelPath( os.path.join(execution_path , "resnet50_coco_best_v2.0.1.h5"))
# detector.loadModel()
# detections = detector.detectObjectsFromImage(input_image=os.path.join(execution_path , "image.jpg"), output_image_path=os.path.join(execution_path , "imagenew.jpg"))
#
# for eachObject in detections:
#     print(eachObject["name"] , " : " , eachObject["percentage_probability"] )

import cv2
import matplotlib.pyplot as plt
import cvlib as cv
from cvlib.object_detection import draw_bbox

im = cv2.imread('image.jpg')
cv.object_detection.populate_class_labels()
bbox, label, conf = cv.detect_common_objects(im, confidence=0.25, model='yolov3-tiny')
output_image = draw_bbox(im, bbox, label, conf)
plt.imshow(output_image)
plt.show()

