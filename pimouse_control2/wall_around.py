#!/usr/bin/env python3
#
# =======================================================================
#   @file   wall_around.py
#   @brief
#   @note
#
#   Copyright (C) 2020 Yasushi Oshima (oosmyss@gmail.com)
# =======================================================================

import rclpy
import copy
import math
import time
from geometry_msgs.msg import Twist
from pimouse_msgs.msg import LightSensorValues
from pimouse_msgs.msg import RunData


class DistanceValues():

    __slots__ = (
        'rightForward', 'rightSide', 'leftSide', 'leftForward', 'leftAverage', 'rightAverage'
    )

    def __init__(self, sensorValues):
        if sensorValues.right_forward > 0:
            self.rightForward = math.sqrt(sensorValues.right_forward)
        else:
            self.rightForward = 0.0
        if sensorValues.right_side > 0:
            self.rightSide = math.sqrt(sensorValues.right_side)
        else:
            self.rightSide = 0.0
        if sensorValues.left_side > 0:
            self.leftSide = math.sqrt(sensorValues.left_side)
        else:
            self.leftSide = 0.0
        if sensorValues.left_forward > 0:
            self.leftForward = math.sqrt(sensorValues.left_forward)
        else:
            self.leftForward = 0.0
        self.leftAverage = 0.0
        self.rightAverage = 0.0

    def SetAverage(self, leftAverage, rightAverage):
        self.leftAverage = leftAverage
        self.rightAverage = rightAverage


class WallAround():

    __slots__ = (
        '_cmdVel', '_accel', '_decel', '_maxSpeed', '_minSpeed', '_servoTarget',
        '_servoKp', '_servoKd', '_leftSideBuffer', '_rightSideBuffer', '_bufferIndex',
        '_servoOffThreshold', '_wallGain', '_wallThreshold', '_nearWallThreshold',
        '_distanceValues', '_leftAverage', '_rightAverage',
        '_linearSpeed', '_angularSpeed', '_previousError', '_previousTime',
        '_startTime', '_x', '_y', '_th', '_pubRunData', '_isServoOn', '_isTurnRight',
        '_rightThreshold', '_leftThreshold', '_subLightSensors', '_nodeHandle')

    def __init__(self, nodeHandle):
        self._nodeHandle = nodeHandle
        self._cmdVel = self._nodeHandle.create_publisher(Twist, '/cmd_vel', 1)
        self._pubRunData = self._nodeHandle.create_publisher(RunData, '/run_data', 1)

        self._nodeHandle.declare_parameter("run_corridor.acceleration", 0.01)
        self._nodeHandle.declare_parameter("run_corridor.deceleration", 0.02)
        self._nodeHandle.declare_parameter("run_corridor.max_speed", 0.3)
        self._nodeHandle.declare_parameter("run_corridor.min_speed", 0.1)
        self._nodeHandle.declare_parameter("run_corridor.servo_target", 14.0)
        self._nodeHandle.declare_parameter("run_corridor.servo_kp", 8.0)
        self._nodeHandle.declare_parameter("run_corridor.servo_kd", 0.16)
        self._nodeHandle.declare_parameter("run_corridor.servo_off_threshold", 4.0)
        self._nodeHandle.declare_parameter("run_corridor.wall_gain", 1.0)
        self._nodeHandle.declare_parameter("run_corridor.wall_threshold", 15.0)
        self._nodeHandle.declare_parameter("run_corridor.near_wall_threshold", 10.0)
        self._nodeHandle.declare_parameter("run_corridor.right_threshold", 21.0)
        self._nodeHandle.declare_parameter("run_corridor.left_threshold", 21.0)

        self._distanceValues = DistanceValues(LightSensorValues())
        self._leftSideBuffer = [0.0, 0.0, 0.0]
        self._rightSideBuffer = [0.0, 0.0, 0.0]
        self._bufferIndex = 0
        self._leftAverage = 0.0
        self._rightAverage = 0.0
        self._subLightSensors = self._nodeHandle.create_subscription(LightSensorValues, '/lightsensors', self.Callback, 1)

    def Callback(self, messages):
        dv = DistanceValues(messages)
        self._leftSideBuffer[self._bufferIndex] = dv.leftSide
        self._rightSideBuffer[self._bufferIndex] = dv.rightSide
        self._bufferIndex += 1
        if self._bufferIndex >= 3:
            self._bufferIndex = 0
        leftAverage = sum(self._leftSideBuffer) / len(self._leftSideBuffer)
        rightAverage = sum(self._rightSideBuffer) / len(self._rightSideBuffer)
        dv.SetAverage(leftAverage, rightAverage)
        self._distanceValues = dv

    def WallFront(self, dv):
        return (dv.leftForward > self._wallThreshold) or (dv.rightForward > self._wallThreshold)

    def NearWall(self, dv):
        return (dv.leftForward > self._nearWallThreshold) or (dv.rightForward > self._nearWallThreshold)

    def TooRight(self, dv):
        return (dv.rightSide > self._rightThreshold)

    def TooLeft(self, dv):
        return (dv.leftSide > self._leftThreshold)

    def Start(self):
        self._startTime = time.time()
        self._previousError = 0.0
        self._previousTime = self._startTime
        self._linearSpeed = 0.0
        self._angularSpeed = 0.0
        self._x = 0.0
        self._y = 0.0
        self._th = 0.0
        self._isServoOn = False
        self._isTurnRight = False
        self._accel = self._nodeHandle.get_parameter("run_corridor.acceleration", 0.01).value
        self._decel = self._nodeHandle.get_parameter("run_corridor.deceleration", 0.02).value
        self._maxSpeed = self._nodeHandle.get_parameter("run_corridor.max_speed", 0.3).value
        self._minSpeed = self._nodeHandle._nodeHandlendle.get_parameter("run_corridor.min_speed", 0.1).value
        self._servoTarget = self._nodeHandle.get_parameter("run_corridor.servo_target", 14.0).value
        self._servoKp = self._nodeHandle.get_parameter("run_corridor.servo_kp", 8.0).value
        self._servoKd = self._nodeHandle.get_parameter("run_corridor.servo_kd", 0.16).value
        self._servoOffThreshold = self._nodeHandle.get_parameter("run_corridor.servo_off_threshold", 4.0).value
        self._wallGain = self._nodeHandle.get_parameter("run_corridor.wall_gain", 1.0).value
        self._wallThreshold = self._nodeHandle.get_parameter("run_corridor.wall_threshold", 15.0).value
        self._nearWallThreshold = self._nodeHandle.get_parameter("run_corridor.near_wall_threshold", 10.0).value
        self._rightThreshold = self._nodeHandle.get_parameter("run_corridor.right_threshold", 21.0).value
        self._leftThreshold = self._nodeHandle.get_parameter("run_corridor.left_threshold", 21.0).value

    def Run(self):
        dv = self._distanceValues

        error = 0.0
        deltaError = 0.0
        nowTime = time.time()
        elapsedTime = nowTime - self._startTime
        deltaTime = nowTime - self._previousTime
        isWallFront = False

        if self.WallFront(dv):
            self._linearSpeed = 0.0
            if self._isServoOn:
                if dv.rightAverage > dv.leftAverage:
                    self._isTurnRight = False
                else:
                    self._isTurnRight = True
            if self._isTurnRight:
                self._angularSpeed = - math.pi * self._wallGain
            else:
                self._angularSpeed = math.pi * self._wallGain
            self._isServoOn = False
            isWallFront = True
        else:
            if self.TooLeft(dv) or self.TooRight(dv) or self.NearWall(dv):
                self._linearSpeed -= self._decel
            else:
                self._linearSpeed += self._accel
            if self._linearSpeed < self._minSpeed:
                self._linearSpeed = self._minSpeed
            elif self._linearSpeed > self._maxSpeed:
                self._linearSpeed = self._maxSpeed

            if ((dv.leftSide < self._servoOffThreshold)
                    and (dv.rightSide < self._servoOffThreshold)):
                self._angularSpeed = 0.0
                self._isServoOn = False
                self._isTurnRight = False
            elif dv.rightAverage > dv.leftAverage:
                error = dv.rightSide - self._servoTarget
                self._angularSpeed = error * self._servoKp * math.pi / 180.0
                if self._isServoOn and self._isTurnRight:
                    deltaError = error - self._previousError
                    self._angularSpeed += deltaError / deltaTime * self._servoKd * math.pi / 180.0
                self._isServoOn = True
                self._isTurnRight = True
            else:
                error = self._servoTarget - dv.leftSide
                self._angularSpeed = error * self._servoKp * math.pi / 180.0
                if self._isServoOn and (not self._isTurnRight):
                    deltaError = error - self._previousError
                    self._angularSpeed += deltaError / deltaTime * self._servoKd * math.pi / 180.0
                self._isServoOn = True
                self._isTurnRight = False

        data = Twist()
        data.linear.x = self._linearSpeed
        data.angular.z = self._angularSpeed
        self._cmdVel.publish(data)

        self._x += self._linearSpeed * math.cos(self._th) * deltaTime
        self._y += self._linearSpeed * math.sin(self._th) * deltaTime
        self._th += self._angularSpeed * deltaTime
        self._previousError = error
        self._previousTime = nowTime

        runData = RunData()
        runData.elapsed_time = elapsedTime
        runData.x = self._x
        runData.y = self._y
        runData.th = self._th
        runData.linear = self._linearSpeed
        runData.angular = self._angularSpeed
        runData.error = error
        runData.delta_error = deltaError
        runData.left_side = dv.leftSide
        runData.left_forward = dv.leftForward
        runData.right_forward = dv.rightForward
        runData.right_side = dv.rightSide
        runData.is_servo_on = self._isServoOn
        runData.is_turn_right = self._isTurnRight
        runData.is_wall_front = isWallFront

        self._pubRunData.publish(runData)

    def SetVelocity(self, linear, angular):
        data = Twist()
        dv = self._distanceValues
        if self.WallFront(dv) and (linear > 0.0):
            data.linear.x = 0.0
        else:
            data.linear.x = linear
        data.angular.z = angular
        self._cmdVel.publish(data)
