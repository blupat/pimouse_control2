#!/usr/bin/env python3
#
# =======================================================================
#   @file   pimouse_control_node.py
#   @brief
#   @note
#
#   Copyright (C) 2020 Yasushi Oshima (oosmyss@gmail.com)
# =======================================================================

from __future__ import print_function

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from wall_around import WallAround
from face_to_face import FaceToFace
from face_detection import FaceDetection
from pimouse_msgs.srv import PiMouseCmd


class PiMouseControl(Node):

    __slots__ = (
        '_srvCmd', '_srvClientOn', '_srvClientOff', '_wallAround',
        '_faceToFace', '_faceDetection', '_isOn', '_isRun',
        '_on', '_run', '_face', '_forward', '_rotation', '_timer')

    def __init__(self):
        super().__init__('pimouse_control_node')
        self._isOn = False
        self._isRun = False
        self._on = False
        self._run = False
        self._face = False
        self._forward = 0.0
        self._rotation = 0.0
        self._srvCmd = self.create_service(PiMouseCmd, 'pimouse_cmd', self.CommandCallback)
        self._srvClientOn = self.create_client(Trigger, 'motor_on')
        self._srvClientOff = self.create_client(Trigger, 'motor_off')
        self._wallAround = WallAround(self)
        self._faceToFace = FaceToFace(self)
        self._faceDetection = FaceDetection(self)
        while not self._srvClientOn.wait_for_service(timeout=1.0):
            self.get_logger().info('waiting...')
        while not self._srvClientOff.wait_for_service(timeout=1.0):
            self.get_logger().info('waiting...')
        self._timer = self.create_timer(0.05, self.Run)

    def CommandCallback(self, req, res):
        if req.on:
            self._on = True
            if req.run:
                self._run = True
                self._face = False
                self._forward = 0.0
                self._rotation = 0.0
            elif req.face:
                self._run = False
                self._face = True
                self._forward = 0.0
                self._rotation = 0.0
            else:
                self._run = False
                self._face = False
                self._forward = req.forward
                self._rotation = req.rotation
        else:
            self._on = False
            self._run = False
            self._face = False
            self._forward = 0.0
            self._rotation = 0.0
            self._srvClientOff.call()
        res.is_ok = True
        return res

    def Run(self):
        try:
            xPosRate = self._faceDetection.Control(self._face)
            if self._on:
                if not self._isOn:
                    self._srvClientOn.call()
                    self._isOn = True
                if self._run:
                    if not self._isRun:
                        self._wallAround.Start()
                        self._isRun = True
                    else:
                        self._wallAround.Run()
                elif self._face:
                    self._faceToFace.Rotate(xPosRate)
                    self._isRun = False
                else:
                    self._wallAround.SetVelocity(self._forward, self._rotation)
                    self._isRun = False
            else:
                if self._isOn:
                    self._wallAround.SetVelocity(0.0, 0.0)
                    self._srvClientOff.call()
                    self._isOn = False
                    self._isRun = False
        except Exception as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)

    control = PiMouseControl()

    rclpy.spin(control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
