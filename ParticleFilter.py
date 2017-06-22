# -*- coding: utf-8 -*-
"""
Created on Tue Jun 20 15:02:26 2017

@author: leo
"""
from math import pi
import random
import numpy as np

class ParticleFilter(object):
    def __init__(self, model,  x0, y0, theta0, phi0, v0, length, N = 500):
        self.errorPos         = 5
        self.errorPhi         = pi/6 
        self.errorOrientation = pi/6
        self.errorVelocity    = 10
        self.N = N
        self.newSamples = int(0.10*N)
        self.p = []
        self.model = model
        
        x0Array = np.random.normal(x0, self.errorPos,N)
        y0Array = np.random.normal(y0, self.errorPos,N)
        theta0Array = np.random.normal(theta0, self.errorOrientation,N)
        phi0Array = np.random.normal(phi0, self.errorPhi,N)
        v0Array = v0*( 1 + np.random.normal(0, self.errorVelocity,N)/100)
        
        for i in range(self.N):
            c = model(theta_0 = theta0Array[i], x_0 = x0Array[i], y_0 = y0Array[i],
                      acc = 1, dPhi = 1*pi/180, v = v0Array[i], phi = phi0Array[i],
                      length = length, tailDistance = 0)
            
            c.setNoise(self.errorPos, self.errorPhi, self.errorOrientation, self.errorVelocity)
            self.p.append(c)
    
    def getPosition(self):
        x = 0.0
        y = 0.0
        for i in range(self.N):
            x += self.p[i].x
            y += self.p[i].y
        return (x / self.N, y / self.N)
    
    def getPhi(self):
        phi = 0.0
        for i in range(self.N):
            # orientation is tricky because it is cyclic. By normalizing
            # around the first particle we are somewhat more robust to
            # the 0=2pi problem
            phi += self.p[i].phi
        return phi / self.N
        
    def getOrientation(self):
        orientation = 0.0
        for i in range(self.N):
            # orientation is tricky because it is cyclic. By normalizing
            # around the first particle we are somewhat more robust to
            # the 0=2pi problem
            orientation += self.p[i].orientation 
        return orientation / self.N
    
    def getVelocity(self):
        v = 0.0
        for i in range(self.N):
            v += self.p[i].v
        return v / self.N
        
    def addData(self, acc, omega, dt, measurements): 
        '''
        Adapting the code for particle filter from the Udacity course artificial inteligence 
        for robotics 
        '''
        # measurements = [ xHat, yHat, orientationHat, phiHat, vHat]

        # --------
        #
        # Update particles
        #     
        # motion update (prediction)

        p2 = []
        xArray = []
        yArray = []
        thetaArray = []
        phiArray = []
        vArray = []
        
        for i in range(self.N):
            currP = self.p[i]
            currP.setAcceleration(acc)
            currP.setOmega(omega)
            currP.updateStates(dt)
            currP.move(dt)
            p2.append(currP)
            xArray = np.append(xArray, currP.x)
            yArray = np.append(yArray, currP.y)
            thetaArray = np.append(thetaArray , currP.orientation)
            phiArray = np.append(phiArray, currP.phi)
            vArray = np.append(vArray, currP.v)
            
        self.p = p2

        # measurement update
        w = []
        for i in range(self.N):
            w.append(self.p[i].measurementProb(measurements))
            
#        print(np.mean(w))
#        print(sum(np.abs(w)))
#        print(np.std(w))
#        print()
        # resampling
        x0Array = np.random.normal(np.mean(xArray), np.std(xArray),self.newSamples)
        y0Array = np.random.normal(np.mean(yArray), np.std(yArray),self.newSamples)
        theta0Array = np.random.normal(np.mean(thetaArray), np.std(thetaArray),self.newSamples)
        phi0Array = np.random.normal(np.mean(phiArray), np.std(phiArray),self.newSamples)
        v0Array = np.mean(vArray)*( 1 + np.random.normal(0, np.std(vArray),self.newSamples)/100)
        
        p3 = []
        for i in range(self.newSamples):
            c = self.model(theta_0 = theta0Array[i], x_0 = x0Array[i], y_0 = y0Array[i],
                          acc = acc, dPhi = omega, v = v0Array[i], phi = phi0Array[i],
                          length = self.p[0].length, tailDistance = 0)
            c.setNoise(self.errorPos, self.errorPhi, self.errorOrientation, self.errorVelocity)
            p3.append(c)
            
        index = int(random.random() * self.N)
        beta = 0.0
        mw = max(w)
        for i in range(self.N-self.newSamples):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % self.N
            p3.append(self.p[index].duplicate())
        self.p = p3
        
        