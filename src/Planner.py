# -*- coding: utf-8 -*-
"""
Created on Sun Jun 18 20:46:04 2017

@author: leo
"""
from math import sin, cos, tan, pi, sqrt, atan, atan2, asin
import numpy as np

class Planner(object):
    '''
    Planner class, that will define the path the car should follow.
    '''

    def __init__(self):

        # Reference coordinates, velocity and angle for the track
        self.x_track = None
        self.y_track = None
        self.angle_track = None
        self.v_track = None

        # Total length of the track
        self.trackLen = 0

        # Keeps tracks of the current goals.
        self.currGoal = 1
        self.nextX = None
        self.nextY = None
        self.nextAngle = None
        self.nextV = None
        self.lastUpdate = 0

		  # Parameters for the algorithms
        self.pathUpdateLookAhead = .2
        self.minPathUpdateDistance = 2
        self.headingWindow = 0

    def setTrack_XY(self, x_ref, y_ref, dt = 1):
        '''
         Set the track based on the X, Y coordinates and the sampling time.
         Calculates the velocity and angle
        '''
        self.x_track = np.array(self.smooth(x_ref))
        self.y_track = np.array(self.smooth(y_ref))

        self.trackLen = self.x_track.size
        self.angle_track = np.empty(self.trackLen )
        self.v_track = np.empty(self.trackLen )

        for i in range(1, self.trackLen):
            dx = self.x_track[i] - self.x_track[i-1]
            dy = self.y_track[i] - self.y_track[i-1]
            self.angle_track[i-1] = atan2(dy, dx)
            self.v_track[i-1] = sqrt(dx*dx + dy*dy)/dt

        self.angle_track[i] = self.angle_track[i-1]
        self.v_track[i] = self.v_track[i-1]


    def setTrack_OrientationVelocity(self, angle_ref, v_ref, dt = 1, x_0 = 0, y_0 = 0):
        '''
         Set the track based on the velocity, orientation and the sampling time. Calculates the
         X and Y coordinates.
        '''
        self.angle_track = np.array(self.smooth(angle_ref))
        self.v_track = np.array(self.smooth(v_ref))

        self.trackLen = self.angle_track.size
        self.x_track = np.empty(self.trackLen)
        self.y_track = np.empty(self.trackLen)

        self.x_track[0] = x_0
        self.y_track[0] = y_0
        for i in range(1, self.trackLen):
            self.x_track[i] = self.x_track[i] + cos(self.angle_track[i-1])*self.v_track[i-1]*dt
            self.y_track[i] = self.y_track[i] + sin(self.angle_track[i-1])*self.v_track[i-1]*dt

    def setTrack(self, x_ref, y_ref, angle_ref, v_ref, smoothTrack = True):
		'''
 		Set the track based on the velocity, orientation, x and y coordinates.
	 	The smoothTrack flag defines if we should apply the smoothing or not.
		'''

		if(smoothTrack):
			self.x_track = np.array(self.smooth(x_ref))
			self.y_track = np.array(self.smooth(y_ref))
			self.angle_track = np.array(self.smooth(angle_ref))
		else:
			self.x_track = np.array(x_ref)
			self.y_track = np.array(y_ref)
			self.angle_track = np.array(angle_ref)

		self.v_track = np.array(v_ref)
		self.trackLen = len(v_ref)

    def smooth(self, path, weight_data = 0.6, weight_smooth = 0.1, tolerance = 0.000001):
        '''
        Using the code for smooth function from the course the artificial inteligence
        for robotics class
        '''
        newpath = list(path)
        totalChange = tolerance
        while totalChange >= tolerance:
            totalChange = 0
            for i in range(1,len(newpath)-1):
                oldValue = newpath[i]
                newpath[i] = newpath[i] + weight_data*(path[i] - newpath[i]) + \
                                weight_smooth*(newpath[i+1] + newpath[i-1] - 2*newpath[i])

                totalChange += abs(oldValue  - newpath[i])
        return newpath

    #def checkForCollision(self):


    def getPath(self, x, y):
        '''
        Given the position of the vehicle, returns the x, y, angle and velocity
        of the track.
        '''
        idx = self.updatePathIndex(x,y)
        return (self.x_track[idx], self.y_track[idx], self.angle_track[idx], self.v_track[idx])

    def updatePathIndex(self, x, y):
        '''
        Gets the index for the the closest point in the path of the current
        position.
        '''
        distance = (self.x_track - x)**2 + (self.y_track - y)**2
        idx = distance.argmin()
#        self.currGoal = max(self.currGoal, idx)
        self.currGoal = idx
        return idx

    def updateGoal(self, x, y, v, dt, adaptativeUpdate = True):
        '''
        Updates the next pos, orientation and velocity
        '''
        if(adaptativeUpdate == True):
            pathUpdateDistance = abs(v*self.pathUpdateLookAhead)
        else:
            pathUpdateDistance = self.minPathUpdateDistance

        if(pathUpdateDistance < self.minPathUpdateDistance):
            pathUpdateDistance = self.minPathUpdateDistance

        lIdx = max(self.currGoal - self.headingWindow , 0)
        rIdx = min(self.currGoal + self.headingWindow , self.trackLen - 1)
        self.nextX = (self.x_track[lIdx] + self.x_track[self.currGoal] + self.x_track[rIdx])/3
        self.nextY = (self.y_track[lIdx] + self.y_track[self.currGoal] + self.y_track[rIdx])/3
        self.nextAngle = (self.angle_track[lIdx] + self.angle_track[self.currGoal] + self.angle_track[rIdx])/3
        self.nextV = self.v_track[self.currGoal]

        distance = sqrt((self.nextX - x)**2 + (self.nextY - y)**2)
        if(distance < pathUpdateDistance):
            distance = (self.x_track - x)**2 + (self.y_track - y)**2
            idx = distance.argmin()
            self.currGoal = min(self.currGoal + 1, self.trackLen - 1)
            self.currGoal = max(self.currGoal, idx)
        else:
            return [self.nextX, self.nextY, self.nextAngle, self.nextV]

        lIdx = max(self.currGoal - self.headingWindow, 0)
        rIdx = min(self.currGoal + self.headingWindow, self.trackLen - 1)
        self.nextX = (self.x_track[lIdx] + self.x_track[self.currGoal] + self.x_track[rIdx])/3
        self.nextY = (self.y_track[lIdx] + self.y_track[self.currGoal] + self.y_track[rIdx])/3
        self.nextAngle = (self.angle_track[lIdx] + self.angle_track[self.currGoal] + self.angle_track[rIdx])/3
        self.nextV = self.v_track[self.currGoal]

        return [self.nextX, self.nextY, self.nextAngle, self.nextV]

    def getGoal(self, x, y, v, dt):
        '''
        Gets the next x, y, orientation and velocity.
        '''
        self.updateGoal(x, y, v, dt)
        return [self.nextX, self.nextY, self.nextAngle, self.nextV]


class SplinePlanner(Planner):
    '''
    Spline Planner class. Given a point in space and an orientation, generates
    spline with the path to it.
    '''

    def __init__(self, xRef, yRef, orientationRef):
        Planner.__init__(self)
        self.xRef = xRef
        self.yRef = yRef
        self.orientationRef = orientationRef




