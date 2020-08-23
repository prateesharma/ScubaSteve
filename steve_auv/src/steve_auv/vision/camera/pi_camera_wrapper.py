####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import time

from picamera import PiCamera, PiRGBArray


class PiCameraWrapper(object):
    """Provides interface for the RPi camera."""

    def __init__(self):
        rospy.loginfo("Creating camera object.")
        self._camera = PiCamera()
        self._image_count = 0

        # Allow time for the camera to warm up
        time.sleep(1.0)

    def capture(self):
        image_raw = PiRGBArray(self._camera)
        self._camera.capture(image_raw, format="bgr")
        self._image_count += 1
        return image_raw.array

    def get_image_count(self):
        return self._image_count
