# -*- coding: utf-8 -*-
"""
Created on Wed Jul 26 16:33:02 2017

@author: leo
"""

import numpy as np

from math import cos, sin, atan, pi

from shapely.geometry import Polygon
from shapely.geometry import Point, LineString, mapping, shape
from shapely.ops import cascaded_union
from shapely import affinity
import timeit

class LIDAR(object):
    def __init__(self, posX, posY, resolution = 1., mapRadius = 100):
        self.x = posX
        self.y = posY
        self.origin = Point(self.x, self.y)
        self.resolution = pi*resolution/180.0
        self.nRays = int(np.floor(2.*pi/self.resolution))
        self.measure = np.zeros(self.nRays)
        self.pointMap = [None]*self.nRays
        self.rays = [None]*self.nRays
        self.angles = np.arange(0, 2.*pi, self.resolution)
        self.mapRadius = mapRadius

    def updatePose(self, delta_x, delta_y, delta_theta):
        self.x += delta_x
        self.y += delta_y
        self.origin = Point(self.x, self.y)
#        self.angles += delta_theta

    def setPose(self, pos, theta):
        self.x = pos[0]
        self.y = pos[1]
        self.origin = Point(self.x, self.y)

        self.angles = np.arange(theta, 2.*pi+theta, self.resolution)

    def updateRays(self):
        for idx in range(self.nRays):
            angle = self.angles[idx]
            endPoint = Point(self.x + cos(angle)*self.mapRadius, self.y + sin(angle)*self.mapRadius)
            self.rays[idx] = LineString([self.origin, endPoint])

    def rayTracing(self, bodies):
#
#        polys = [b.getPolygon() for b in bodies]
#        radial_sweep = cascaded_union(self.rays)
#        all_input_lines = cascaded_union(polys)
#        self.measure = np.repeat(self.mapRadius,self.nRays)
#        self.pointMap = [None]*self.nRays
#        for idx in range(len(radial_sweep)):
#            radial_line = radial_sweep[idx]
#            inter = radial_line.intersection(all_input_lines)
#            if inter.is_empty:
#               continue
#
#            if len(inter.coords) > 1:
#               # ray intersects at multiple points
#               inter_dict = {}
#               for inter_pt in inter.coords:
#                   p = Point(inter_pt)
#                   inter_dict[self.origin.distance(p)] = p
#               distance = min(inter_dict.keys())
#               intersect = inter_dict[distance]
#            else:
#               # ray intersects at one point only
#               distance  = self.origin.distance(inter)
#               intersect = inter
#
##            if(distance < self.mapRadius):
##                if(bodyNotFound == True):
##                    # it's the first sign of this body. Let's limit the scan
##                    if(distance > body.radius):
##                        alpha = asin(body.radius/distance) #angle left for scanning
##                        lastRayIdx = min(lastRayIdx, idx + int(alpha/self.resolution))
##                bodyNotFound = False
#
#            if(self.measure[idx] > distance):
#                self.measure[idx] = distance
#                self.pointMap[idx] = intersect
#
        closeBodies = []
        for body in bodies:
            if self.origin.distance(body.getOrigin()) <= self.mapRadius + body.radius:
                closeBodies.append(body)

        self.measure = np.repeat(self.mapRadius,self.nRays)
        self.pointMap = [r.boundary[1] for r in self.rays]

        for body in closeBodies:
            lastRayIdx = self.nRays
            idx = 0
            bodyNotFound = True
            poly = body.getPolygon()
            while idx < lastRayIdx:
                ray = self.rays[idx]
                distance = self.mapRadius
                inter = poly.intersection(ray)

                if inter.is_empty:
                   # radial line doesn't intersect, so skip
                   idx += 1
                   continue

                if len(inter.coords) > 1:
                   # ray intersects at multiple points
                   inter_dict = {}
                   for p in inter.boundary:
                       inter_dict[self.origin.distance(p)] = p
                   distance = min(inter_dict.keys())
                   intersect = inter_dict[distance]
                else:
                   # ray intersects at one point only
                   distance  = self.origin.distance(inter)
                   intersect = inter

                if(distance < self.mapRadius and bodyNotFound == True):
                    # it's the first sign of this body. Let's limit the scan
                    if(distance > body.radius):
                        alpha = atan((2.*body.radius)/distance) #angle left for scanning
                        lastRayIdx = min(lastRayIdx, idx + int(alpha/self.resolution))
                    bodyNotFound = False

                if(self.measure[idx] > distance):
                    self.measure[idx] = distance
                    self.pointMap[idx] = intersect
                idx += 1