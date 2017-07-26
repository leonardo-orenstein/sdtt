# -*- coding: utf-8 -*-
"""
Created on Wed Jul 12 19:03:06 2017

@author: leo
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Arrow
from math import sin, cos, tan, pi, exp, sqrt
import numpy as np
from shapely.geometry import Point
from shapely.geometry import Polygon
from shapely.geometry import LineString

class Body(object):
    '''
    Represent a solid body by a list of vertices, the position of its center of
    mass and orientation
    '''

    def __init__(self, orientation = 0, x = 0, y = 0, v = 0, acc = 0, omega = 0, vertex = None):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.v   = v
        self.acc = acc
        self.omega = omega

        self.lastUpdate = 0

        if vertex is None:
            self.vertices = np.matrix([0,0])
        else:
            self.vertices = np.matrix(vertex)

        self.radius = self.__calculateRadius__()

        # handling plot
        self.plotRefreshRate = 2 # seconds
        self.fig = None
        self.timeOfLastPlot = -self.plotRefreshRate

        self.bodyLine = None
        self.directionArrow = None

    @classmethod
    def fromVehicle(cls, vehicle):
        orientation = vehicle.getOrientation()
        pos = vehicle.getPos()
        vel = vehicle.getVelocity()
        acc = vehicle.getAcc()
        omega = vehicle.getOmega()

        vertices = [(-vehicle.length/2 , -2),
                    (-vehicle.length/2 , 2),
                    (vehicle.length/2 , 2),
                    (vehicle.length/2 , -2)]
        return cls(orientation=orientation, x=pos[0], y=pos[1], v = vel, acc = acc, omega = omega, vertex = vertices)

    def setVertex(self, vertex):
        self.vertices = np.matrix(vertex)
        self.radius = self.__calculateRadius__()

    def __calculateRadius__(self):
        rows, cols = self.vertices.shape
        radius = 0
        for i in range(rows):
            currRadius = sqrt(self.vertices[i,]*self.vertices[i,].T)
            radius = max(radius, currRadius)
        return radius

    def checkBodyCollision(self, body):
        d = self.getDistance(body)
        if(d < self.radius+body.radius):
            polygonSelf = Polygon(self.getPose())
            polygonBody = Polygon(body.getPose())
            return polygonSelf.intersects(polygonBody)
        else:
            return False

    def checkLineCollision(self, line):
        polygonSelf = Polygon(self.getPose())
        return polygonSelf.intersects(line)

    def getDistance(self, body):
        return sqrt((self.x - body.x)**2 + (self.y- body.y)**2)

    def setAcceleration(self, acc):
        self.acc = acc

    def setOmega(self, omega):
        self.omega= omega

    def updateStates(self, t):
        dt = t - self.lastUpdate

        self.x += self.v*cos(self.orientation)
        self.y += self.v*sin(self.orientation)
        self.v += self.acc*dt
        self.orientation += self.omega*dt

        self.lastUpdate = t

    def setStates(self, x, y, v, orientation, t):
        self.x = x
        self.y = y
        self.v = v
        self.orientation = orientation

        self.lastUpdate = t

    def resetTime(self):
        self.lastUpdate = 0

    def getPosition(self):
        return self.x, self.y

    def getOrientation(self):
        return self.orientation

    def getVelocity(self):
        return self.v

    def createPlot(self, fig = None, ax = None):
        self.timeOfLastPlot = -1

        if(fig is None):
            self.fig = plt.figure()
        else:
            self.fig = fig

        if(ax is None):
            self.ax = self.fig.add_subplot(111)
        else:
            self.ax = ax

        (verticesX, verticesY) = self.getDrawingVertex()
        self.bodyLine, = self.ax.plot(verticesX, verticesY, 'k')
        self.directionArrow = Arrow(self.x, self.y,
                                     0.1*self.v*cos(self.orientation),
                                     0.1*self.v*sin(self.orientation),
                                     color = 'c')
        self.ax.add_patch(self.directionArrow)
        self.ax.set_xlim([self.x - 10, self.x + 10])
        self.ax.set_ylim([self.y - 10, self.y + 10])
        self.ax.set_xlabel('Distance X')
        self.ax.set_ylabel('Distance Y')

    def getPose(self):
        verticesX = []
        verticesY = []
        self.R = np.matrix([(cos(self.orientation), -sin(self.orientation)), (sin(self.orientation), cos(self.orientation))])
        v = self.R*self.vertices.transpose() # applyting the rotation matrix
        v = v.transpose()
        rows, cols = v.shape
        for i in range(rows):
            verticesX.append(v[i,0] + self.x)
            verticesY.append(v[i,1] + self.y)
        return v + np.matrix([self.x, self.y])

    def getDrawingVertex(self):
        vertices = self.getPose()
        vX = [v[0,0] for v in vertices]
        vX.append(vertices[0,0])
        vY = [v[0,1] for v in vertices]
        vY.append(vertices[0,1])
        return vX, vY

    def plot(self, draw = True):
        (verticesX, verticesY) = self.getDrawingVertex()
        self.bodyLine.set_ydata(verticesY)
        self.bodyLine.set_xdata(verticesX)

        self.directionArrow.remove()
        self.directionArrow = Arrow(self.x, self.y,
                                     0.1*self.v*cos(self.orientation),
                                     0.1*self.v*sin(self.orientation),
                                     color = 'c')
        self.ax.add_patch(self.directionArrow)
        self.ax.set_xlim([self.x - 10, self.x + 10])
        self.ax.set_ylim([self.y - 10, self.y + 10])

        if(draw):
            self.fig.canvas.draw()
            plt.pause(0.001)
