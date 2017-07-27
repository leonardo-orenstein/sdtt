# -*- coding: utf-8 -*-
"""
Created on Sun Jun 18 20:25:02 2017

@author: leo
"""
import cv2
import numpy as np
from collections import namedtuple
from math import cos

CamParams = namedtuple("CamParams", "height width heightCalibration widthCalibration leftEdge rightEdge px2m y_1Calibration y_2Calibration y_3Calibration y_4Calibration x_1Calibration x_2Calibration x_3Calibration x_4Calibration dist mtx")

class Sensor(object):
    '''
    The sensor class is responsible to acquire data from the enviroment
    '''

    def __init__(self):
        self.measure = None
        self.scanner = None
        self.uncertanty = None # This is the expected uncertanty for this sensor
        self.proportionalUncertanty = None # if true the uncertnaty is processed as a % of the measure

    def connect(self, scanner):
        self.scanner = scanner

    def scan(self):
        self.measure = self.scanner()

    def read(self):
        return self.measure

    def setUncertanty(self, uncertanty, proportionalUncertanty = False):
        self.uncertanty = uncertanty # This is the expected uncertanty for this sensor
        self.proportionalUncertanty = proportionalUncertanty # if true the uncertnaty is processed as a % of the measure

    def getUncertanty(self):
        if(self.proportionalUncertanty):
            return max(0.01, abs(self.measure*self.uncertanty)) # setting a lower boundary for uncertanty
        else:
            return self.uncertanty

class SensorCamera(Sensor):
    def __init__(self, params):
        self.height = params.height
        self.width = params.width
        heightCalibration = params.heightCalibration
        widthCalibration = params.widthCalibration
        self.leftEdge   = params.leftEdge
        self.rightEdge  = params.rightEdge
        self.px2m       = params.px2m
        self.threshold = 150.0
        self.laneWidth = 0.25 #meters
        self.dist = params.dist
        self.mtx = params.mtx
        self.frame = None

        dest =  np.float32([[0,0],[0,self.height],[self.width,0],[self.width,self.height]])

        yRatio = np.float32(self.height)/heightCalibration
        y_1 = np.int64(params.y_1Calibration*yRatio)
        y_2 = np.int64(params.y_2Calibration*yRatio)
        y_3 = np.int64(params.y_3Calibration*yRatio)
        y_4 = np.int64(params.y_4Calibration*yRatio)

        xRatio = np.float32(self.width)/widthCalibration
        x_1= np.int64(params.x_1Calibration*xRatio)
        x_2 = np.int64(params.x_2Calibration*xRatio)
        x_3 = np.int64(params.x_3Calibration*xRatio)
        x_4 = np.int64(params.x_4Calibration*xRatio)

        areaOfInterest = np.float32([[x_1, y_1], [x_2, y_2] , [x_3, y_3], [x_4, y_4]])

        self.M = cv2.getPerspectiveTransform(areaOfInterest, dest)
        self.Minv = cv2.getPerspectiveTransform(dest, areaOfInterest)


####
    def processFrame(self, frame):
        warped = cv2.warpPerspective(frame, self.M, (self.width, self.height))

#        imgGray = cv2.cvtColor(warped, cv2.COLOR_RGB2GRAY)
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
#        lower_blue = np.array([40,40,40])
        lower_blue = np.array([40,15,15])
        upper_blue = np.array([140,255,255])

        imgGray = cv2.inRange(hsv, lower_blue, upper_blue)


        midPoint = np.int64(self.width/2)
        leftSide = np.zeros((self.height,midPoint))
        for r in range(self.height):
            for c in range(midPoint):
                leftSide[r][c] = imgGray[r][c]

        rightSide = np.zeros((self.height,midPoint))
        for r in range(self.height):
            for c in range(midPoint,np.int64(self.width)):
                rightSide[r][c-midPoint] = imgGray[r][c]

        yLeft, xLeft = np.where(leftSide > self.threshold)
        yRight, xRight= np.where(rightSide > self.threshold)
        xRight += midPoint

        xArrayLeft, yArrayLeft, angleArrayLeft = self.findLanes(xLeft, yLeft, self.height, 1)
        xArrayRight, yArrayRight, angleArrayRight = self.findLanes(xRight, yRight, self.height, 1)

        return ((xArrayLeft, yArrayLeft, angleArrayLeft), (xArrayRight, yArrayRight, angleArrayRight))

    def getPosition(self, frame):
        ((xArrayLeft, yArrayLeft, angleArrayLeft), (xArrayRight, yArrayRight, angleArrayRight)) = self.processFrame(frame)

        midPoint = np.int64(self.width/2)

        distanceToLeftLane = (midPoint - xArrayLeft[0])*cos(angleArrayLeft[0])*self.px2m
        distanceToRightLane = (xArrayRight[0] - midPoint)*cos(angleArrayRight[0])*self.px2m
#
#        print distanceToLeftLane
#        print angleArrayLeft[0]
#        print '//'
#        print distanceToRightLane
#        print angleArrayRight[0]
#        print
        return (distanceToLeftLane + (self.laneWidth - distanceToRightLane))/2
#        return (xArrayRight[0] - xArrayLeft[0])*self.px2m/2


    def scan(self):
        frame = self.scanner()
        self.frame = cv2.undistort(frame, self.mtx, self.dist, None, self.mtx)
#        self.frame = frame
        self.measure = self.getPosition(self.frame)
#
#    def scanner(self):
##        return cv2.imread('C:\Users\leo\Documents\sdtt\solidYellowLeft.jpg')
#        return cv2.imread('C:\Users\leo\Documents\sdtt\img.jpeg')

    def findLanes(sefl, x, y, yLen, segments = 1):
        N = segments+1
        order = 2
        assert(segments <= order)
        p = np.polyfit(y,x,order)
        xArray = np.zeros(N)
        xArray[0] = np.int64(np.mean(x[0:10]))

        yArray = np.zeros(N)
        yArray[0] = 0

        angleArray = np.zeros(N)
        angleArray[0] = p[1]

        for i in range(1,N):
            yArray[i] = np.int64((i+1)*yLen/N)
            xArray[i] = np.int64(p[0]*yArray[i]**2 + p[1]*yArray[i] + p[2])
            angleArray[i] = p[0]*yArray[i]*2 + p[1]

        return xArray, yArray, angleArray

    def draw(self):
        frame = self.frame
        ((xArrayLeft, yArrayLeft, _), (xArrayRight, yArrayRight, _)) = self.processFrame(frame)
        self.__draw__(frame, xArrayLeft, yArrayLeft)
        self.__draw__(frame, xArrayRight, yArrayRight)
        cv2.imshow('Camera',frame)

    def __draw__(self, frame, xArray, yArray):
        i = 0
        pts = np.array([np.array([[xArray[i],yArray[i]]], dtype = "float32")])
        x1, y1 = cv2.perspectiveTransform(pts, self.Minv)[0][0]
        for i in range(1,xArray.size):
            pts = np.array([np.array([[xArray[i],yArray[i]]], dtype = "float32")])
            x2, y2 = cv2.perspectiveTransform(pts,self.Minv)[0][0]
            cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),10)
            x1, y1 = x1, y2

        return frame, xArray, yArray
