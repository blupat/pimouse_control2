#!/usr/bin/env python3
#
# =======================================================================
#   @file   face_detection.py
#   @brief
#   @note
#
#   Copyright (C) 2020 Yasushi Oshima (oosmyss@gmail.com)
# =======================================================================

import rclpy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class FaceDetection():

    __slots__ = (
        '_pubFace', '_cvBridge', '_imageOrg', '_cascade',
        '_subImage', '_nodeHandle')

    def __init__(self, nodeHandle):
        self._subImage = nodeHandle.create_subscription(Image, "/image_raw", self.GetImage, 1)
        self._pubFace = nodeHandle.create_publisher(Image, "face", 1)
        self._cvBridge = CvBridge()
        self._imageOrg = None

        self._nodeHandle = nodeHandle
        self._nodeHandle.declare_parameter("vision_control.scale_factor", 1.3)
        self._nodeHandle.declare_parameter("vision_control.min_neighbors", 2)

        classifier = "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"
        self._cascade = cv2.CascadeClassifier(classifier)

    def GetImage(self, img):
        try:
            self._imageOrg = self._cvBridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(e)

    def Monitor(self, rect, org):
        if rect is not None:
            cv2.rectangle(org, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 255, 255), 4)

        self._pubFace.publish(self._cvBridge.cv2_to_imgmsg(org, "bgr8"))

    def DetectFace(self, isDetectionEnabled):
        if self._imageOrg is None:
            return None, None

        org = self._imageOrg

        if isDetectionEnabled:
            scaleFactor = self._nodeHandle.get_parameter("vision_control.scale_factor").value
            minNeighbors = self._nodeHandle.get_parameter("vision_control.min_neighbors").value
            gimg = cv2.cvtColor(org, cv2.COLOR_BGR2GRAY)
            face = self._cascade.detectMultiScale(
                gimg, scaleFactor, minNeighbors, cv2.CASCADE_FIND_BIGGEST_OBJECT)
        else:
            face = []

        if len(face) == 0:
            self.Monitor(None, org)
            return None, org

        r = face[0]
        self.Monitor(r, org)

        return r, org

    def Control(self, isDetectionEnabled):
        r, image = self.DetectFace(isDetectionEnabled)
        if r is None:
            return 0.0

        wid = image.shape[1] / 2
        xPosRate = (r[0] + r[2] / 2 - wid) * 1.0 / wid
        return xPosRate
