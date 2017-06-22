# -*- coding: utf-8 -*-
"""
Created on Thu Jun 22 11:02:21 2017

@author: leo
"""
from math import sin, cos, pi

class Track(object):
    def __init__(self, segments, dt):
        self.vel = []
        self.angles = []

        self.x = []
        self.y = []
        
        self.xUpper = []
        self.yUpper = []
        
        self.xLower = []
        self.yLower = []
        
        self.xCenterLane = []
        self.yCenterLane = []
        
        x0, y0 = (0, 0)
        for s in segments:
            self.vel.extend(s.vel)
            self.angles.extend(s.angles)
            s.createXY(dt, x0, y0)    
            self.x.extend(s.x)
            self.y.extend(s.y)
            
            self.xUpper.extend(s.xUpper)
            self.yUpper.extend(s.yUpper)
            
            self.xLower.extend(s.xLower)
            self.yLower.extend(s.yLower)
            
            self.xCenterLane.extend(s.xCenterLane)
            self.yCenterLane.extend(s.yCenterLane)
            x0 = self.x[-1]
            y0 = self.y[-1]
            
        self.N = len(self.vel)
        
        def createXY(self, dt, x0=0, y0=0):
            for i in len(self.N):
                self.x[i] += x0
                self.y[i] += y0
                
                self.xUpper[i] += x0
                self.yUpper[i] += y0
                
                self.xLower[i] += x0
                self.yLower[i] += y0
                
                self.xCenterLane[i] += x0
                self.yCenterLane[i] += y0
            
class Segment(Track):
    def __init__(self, vel, angles):
        assert(len(vel) == len(angles))
        self.vel = vel
        self.angles = angles
        self.N = len(self.vel)
        self.numberOfLanes = 2
        self.roadWidth = 7

        
    def createXY(self, dt, x0 = 0, y0 = 0):
        self.x = [x0]
        self.y = [y0]
        for i in range(1,self.N):
            self.x.append(self.vel[i]*dt*cos(self.angles[i]) + self.x[i-1])
            self.y.append(self.vel[i]*dt*sin(self.angles[i]) + self.y[i-1])

        self.xUpper = []
        self.yUpper = []
        
        self.xLower = []
        self.yLower = []
        
        self.xCenterLane = []
        self.yCenterLane = []
        
        for i in range(0,self.N):
            self.xCenterLane.append(self.x[i] + sin(-self.angles[i])*self.roadWidth/4)
            self.yCenterLane.append(self.y[i] + cos(-self.angles[i])*self.roadWidth/4)
            
            self.xUpper.append(self.x[i] + sin(-self.angles[i])*self.roadWidth*3/4)
            self.yUpper.append(self.y[i] + cos(-self.angles[i])*self.roadWidth*3/4)
            
            self.xLower.append(self.x[i] - sin(-self.angles[i])*self.roadWidth/4)
            self.yLower.append(self.y[i] - cos(-self.angles[i])*self.roadWidth/4)
            
class StraightSegment(Segment):
    def __init__(self, vel, angle):
        angles = [angle for _ in vel]
        Segment.__init__(self, vel, angles)
        
class ArcCurveSegment(Segment):
    def __init__(self, vel, radius, entryAngle, dt):
        angles = [entryAngle for _ in vel] # creates the angles array. Now we got curve it!
        N = len(vel)
        for i in range(1,N):
            omega = vel[i-1]/radius
            angles[i] = (omega*dt + angles[i-1]) % (2*pi)
            if(angles[i] > pi):
                angles[i] -= 2*pi
        Segment.__init__(self, vel, angles)
        
        
        
        