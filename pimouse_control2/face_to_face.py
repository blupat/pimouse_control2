#!/usr/bin/env python3
#
# =======================================================================
#   @file   face_to_face.py
#   @brief
#   @note
#
#   Copyright (C) 2020 Yasushi Oshima (oosmyss@gmail.com)
# =======================================================================

import rclpy
import math
import sys
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class FaceToFace():

    __slots__ = ('_cmd_vel', '_nodeHandle')

    def __init__(self, nodeHandle):
        self._cmd_vel = nodeHandle.create_publisher(Twist, '/cmd_vel', 1)
        self._nodeHandle = nodeHandle
        self._nodeHandle.declare_parameter("vision_control.angular_gain", 0.3)

    def Rotate(self, xPosRate):
        angular_gain = self._nodeHandle.get_parameter("vision_control.angular_gain").value
        rot = -angular_gain * xPosRate * math.pi

        m = Twist()
        m.linear.x = 0.0
        m.angular.z = rot
        self._cmd_vel.publish(m)
