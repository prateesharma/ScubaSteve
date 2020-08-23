####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import actionlib
import rospy
import steve_auv.gnc as gnc
import steve_auv.vision as vision
import time

from steve_auv.msg import VisionAction, VisionResult
from vision import PiCameraWrapper


class VisionServer(object):
    """Vision action server that listens for goals from the mission manager and
    acts upon them when received. The vision server can work on one of the
    following processes at a time:

        'poi': Periodically takes a picture and determines if there is a point of
               interest (POI) in it. If so, it saves the image as well as the
               location of the POI.

    Processes can be preempted by the mission manager.
    """
    def __init__(self, name, mode_topic, output_dir, rate=10):
        self._name = name
        self._server = actionlib.SimpleActionServer(
                           mode_topic,
                           VisionAction,
                           execute_cb=self.execute_cb
                       ).start()
        self._output_dir = output_dir
        self._rate = rate
        self._camera = PiCameraWrapper()
        self._poi_image_count = 0

    def execute_cb(self, goal):
        is_success = False
        result = VisionResult(start_idx=self._image_count)
        if goal.action == "poi":
            rate = rospy.Rate(self._rate)
            while True:
                if self._server.is_preempt_requested():
                    break

                # 1) Capture the image with the camera
                image = PiCameraWrapper.capture()

                # 2) Run Hough transform
                # TODO

                # 3) If POI, localize
                # TODO

                # 4) If POI, write image to output dir, increment count
                # TODO
                rate.sleep()
        else:
            rospy.logerr("Invalid goal received. Cancelling goal.")
            self._server.set_preempted(result)

        # Return the number of POI images written to output
        result.end_idx = self._image_count

        if is_success:
            self._server.set_succeeded(result)
        else:
            self._server.set_preempted(result)
