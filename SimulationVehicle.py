# -*- coding: utf-8 -*-
"""
Created on Sun Jun 18 20:26:28 2017

@author: leo
"""
from math import sin, cos, tan, pi
import matplotlib.pyplot as plt
import scipy.integrate as integrate


class SimulationVehicle(object):
    '''
    The simulation class has the dynamics of the vehicle for simulation
    '''
    def __init__(self, theta_0 = 0, x_0 = 0, y_0 = 0,  acc = 1, dPhi = 1*pi/180, v = 0, phi = 0):
        self.orientation = theta_0
        self.x = x_0
        self.y = y_0
        
        self.acc = acc
        self.omega = dPhi
        
        self.v   = v
        self.phi = phi
        
        self.lastUpdate = 0
        
    def setAcceleration(self, acc):
        self.acc = acc
        
    def setOmega(self, omega):
        self.omega= omega
        
    def updateStates(self, t):
        dt = t - self.lastUpdate

        self.v   += self.acc*dt
        self.phi += self.omega*dt
        
        self.lastUpdate = t
        
    def resetTime(self):
        self.lastUpdate = 0
        
    def getPosition(self):
        return self.x, self.y
    
    def getSteering(self):
        return self.phi
    
    def getOrientation(self):
        return self.orientation
        
    def getVelocity(self):
        return self.v
       
class Bike(SimulationVehicle):
    def __init__(self, 
                 theta_0 = 0, x_0 = 0, y_0 = 0,
                 acc = 1, dPhi = 1*pi/180, v = 0, phi = 0,
                 length = 2, tailDistance = 0.5):
        SimulationVehicle.__init__(self, theta_0, x_0, y_0, acc, dPhi, v, phi)
        
        self.length = length # length of robot
        self.CoM = length/2 
        self.acc_cent = 0
        self.radius = 0        
        
        self.tailDistance = tailDistance
        
    def set_pose(self, orientation, x, y):
        orientation = orientation % (2*pi)
            
        if(orientation > pi):    
            orientation-= 2*pi
            
        self.orientation = orientation
        self.x = x
        self.y = y
        
    def move(self, motion): 

        steering = motion[0]
        distance = motion[1]
        
        #forward = random.gauss(forward, self.distance_noise)
        #steering= random.gauss(steering, self.steering_noise)
        
        beta = distance/self.length*tan(steering)
        
        if(beta > 0.001):
            R = distance/beta
            
            dx = - sin(self.orientation)*R + sin(self.orientation + beta)*R
            dy = + cos(self.orientation)*R - cos(self.orientation + beta)*R
        
            dRotation = [ self.CoM*(cos(self.orientation + beta) - cos(self.orientation)),
                          self.CoM*(sin(self.orientation + beta) - sin(self.orientation))]

 
            dTranslation = [dx - dRotation[0],
                            dy - dRotation[1]] 
        else:
            dx = cos(self.orientation)*distance
            dy = sin(self.orientation)*distance
            
            dRotation = [0,0]
            dTranslation = [dx, dy]
        
        x = self.x + dx
        y = self.y + dy
        orientation = (self.orientation + beta) % (2*pi)
        
        self.set_pose(orientation, x, y)
        
        
        return ([dRotation, dTranslation], beta)
    
    def print(self):
        print(self.x)
        print(self.y)
        print(self.orientation)
        

        
class BikeTrailer(object):
    def __init__(self, theta = 0, x_0 = 0, y_0 = 0, lengthTractor = 4, lengthTrailer = 10, tailDistance = 0.5):
        self.tractor = Bike(theta, x_0, y_0, 
                            acc = 1, dPhi = 1*pi/180, v = 0, phi = 0,
                            length = lengthTractor, tailDistance = tailDistance)

        self.trailer = Bike(theta, 
                            self.tractor.x - cos(self.tractor.orientation)*(self.tractor.length + self.tractor.tailDistance),
                            self.tractor.y - sin(self.tractor.orientation)*(self.tractor.length + self.tractor.tailDistance),
                            acc = 1, dPhi = 1*pi/180, v = 0, phi = 0,
                            length = lengthTrailer, tailDistance = tailDistance)
        
        # handling plot
        self.plotRefreshRate = 2 # seconds
        self.fig = None
        self.timeOfLastPlot = -self.plotRefreshRate
        
        self.tractorFront = None
        self.trailerFront = None
        self.tractorLine = None
        self.trailerLine = None
        
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
            
        self.tractorFront, = self.ax.plot(self.tractor.x, self.tractor.y, 'r*')
        self.trailerFront, = self.ax.plot(self.trailer.x, self.trailer.y, 'm*')
        self.tractorLine, = self.ax.plot([self.tractor.x, self.trailer.x], [self.tractor.y, self.trailer.y], 'k')
        self.trailerLine, = self.ax.plot([self.tractor.x, self.trailer.x], [self.tractor.y, self.trailer.y], 'b')
        self.ax.set_xlim([-50, 50])
        self.ax.set_ylim([-50, 50])
        self.ax.set_xlabel('Distance X')
        self.ax.set_ylabel('Distance Y')
        
    def calculateTracker(self, dt):
        return self.driver.calculateTracker(self.tractor.x, self.tractor.y, self.tractor.orientation, 
                                               self.tractor.cruiseControl.phi, dt)  

    def calculateAdvancedTracker(self, dt):
        return self.driver.calculateAdvancedTracker(self.tractor.x, self.tractor.y, self.tractor.orientation, 
                                               self.tractor.cruiseControl.phi, self.tractor.length, 
                                               self.tractor.cruiseControl.v, dt)  

        
    def cteLoop(self, dt):
        return self.driver.cteLoop(self.tractor.x, self.tractor.y, dt)  
     
    def backsteppingControl(self, dt):
        return self.driver.backsteppingControl(self.tractor.x, self.tractor.y, self.tractor.orientation, 
                                               self.tractor.cruiseControl.phi, self.tractor.length, dt)
    
    def centripetalLoop(self, max_lat_accel):
        acc_cent    = self.tractor.acc_cent
        v_1         = self.tractor.cruiseControl.v
        phi_1       = self.tractor.cruiseControl.phi
        l_1         = self.tractor.length
        omega       = self.tractor.cruiseControl.omega
        dV          = self.tractor.cruiseControl.acc
        
        return self.driver.centripetalLoop(acc_cent, max_lat_accel, l_1, phi_1, v_1, dV, omega)
    
    def update_posture(self, X):
        l_1   = self.tractor.length
        l_off = self.tractor.tailDistance
        
        x_1, y_1, delta_1, delta_2 = X
        
        x_2 = x_1 - (l_1 + l_off)*cos(delta_1)
        y_2 = y_1 - (l_1 + l_off)*sin(delta_1)    
        
        self.tractor.set_pose(delta_1, x_1, y_1)        
        self.trailer.set_pose(delta_2, x_2, y_2)

    def getStates(self):
        return [self.tractor.x, self.tractor.y, self.tractor.orientation, self.trailer.orientation]
    
    def setStates(self, tractorX, tractorY, tractorOrientation, trailerOrientation):
        self.tractor.x = tractorX
        self.tractor.y = tractorY
        self.tractor.orientation = tractorOrientation
        self.trailer.orientation = trailerOrientation
        
    def move(self, finalTime):
        X0 = self.getStates()
        self.tractor.resetTime()
    
        X = integrate.odeint(self.diff, X0, [0, finalTime])

        self.update_posture(X[-1])
                
        return X
    
    def print(self):
        print('Tractor pose')
        self.tractor.print()
        
        print('Trailer pose')
        self.trailer.print()
        
        print('Velocity')
        print(self.tractor.cruiseControl.v)
        
        print('angle')
        print(self.tractor.cruiseControl.phi)
        
    def plot(self):
        self.tractorFront.set_ydata(self.tractor.y)
        self.tractorFront.set_xdata(self.tractor.x)
        self.tractorLine.set_ydata([self.tractor.y, self.trailer.y])
        self.tractorLine.set_xdata([self.tractor.x, self.trailer.x])

        self.trailerFront.set_ydata(self.trailer.y)
        self.trailerFront.set_xdata(self.trailer.x)
        
        x_4 = self.trailer.x - (self.trailer.length + self.trailer.tailDistance)*cos(self.trailer.orientation)
        y_4 = self.trailer.y - (self.trailer.length + self.trailer.tailDistance)*sin(self.trailer.orientation)
        self.trailerLine.set_ydata([y_4, self.trailer.y])
        self.trailerLine.set_xdata([x_4, self.trailer.x])
        
        self.ax.set_xlim([self.trailer.x - 50, self.trailer.x + 50])
        self.ax.set_ylim([self.trailer.y - 50, self.trailer.y + 50])
        
        self.fig.canvas.draw()
        plt.pause(0.01)    
            
    def diff(self, X, t):
        delta_1 = X[2]
        delta_2 = X[3]
        self.update_posture(X)

        self.tractor.updateStates(t)
        
        phi_1 = self.tractor.phi
        v_1 = self.tractor.v
        
        l_1 = self.tractor.length
        l_2 = self.trailer.length
        l_off = self.tractor.tailDistance
        
        theta_1 = delta_1 + phi_1
        dDelta_1 =  (1/l_1)*(v_1*sin(phi_1))
        
        if(abs(phi_1 ) > 0 ):
            self.radius = l_1/sin(phi_1)
            self.tractor.acc_cent = v_1*v_1/self.radius 
        else:
            self.tractor.acc_cent = 0
            
        alpha = delta_1 - delta_2
        
        c_1 = v_1*sin(phi_1)
        c_2 = v_1*cos(phi_1)
        c_3 = sin(alpha)
        c_4 = cos(alpha)
        
        dDelta_2 = (1/l_2)*(c_2*c_3) - (l_off/(l_1*l_2))*c_1*c_4
        
        dX_1 = v_1*cos(theta_1)
        dY_1 = v_1*sin(theta_1)
        
        self.setStates(dX_1, dY_1, dDelta_1, dDelta_2)

        if( t - self.timeOfLastPlot >= self.plotRefreshRate):
            self.plot()
            self.timeOfLastPlot = t
            
        return [dX_1, dY_1, dDelta_1, dDelta_2]      