# -*- coding: utf-8 -*-
"""
Created on Thu Jun  8 13:13:41 2017

@author: leo

implementing the tractor - trailer model from:
    Trailer Steering Control of a Tractor–Trailer Robot
    http://www.dct.tue.nl/New/Wouw/CST2016_Ritzen.pdf

For simplicity sake considering only the kinematics    
"""

## Bike trailer model
from math import sin, cos, tan, pi, sqrt, atan, atan2, asin
import matplotlib.pyplot as plt
import numpy as np
import scipy.integrate as integrate

    
class CruiseControl(object):
    def __init__(self, acc = 1, dPhi = 1*pi/180):
        self.acc = acc
        self.omega = dPhi
        
        self.v   = None
        self.phi = None
        
        self.v_ref = None
        self.phi_ref = None
        
        self.lastUpdate = 0
        self.tolerance = 0.001
              
        self.useCTELoop = False
        
    def setVelocityRef(self, vRef):
        self.v_ref = vRef
        
    def setPhiRef(self, phiRef):
        self.phi_ref = phiRef

    def setAcceleration(self, acc):
        self.acc = acc
        
    def setOmega(self, omega):
        self.omega= omega
        
    def setVelocity(self, v):
        self.v = v
    
    def setPhi(self, phi):
        self.phi = phi
        
    def updateStates(self, t):
        if(self.useCTELoop == True):
            if(self.phi_ref - self.phi > self.tolerance):
                dPhi = self.omega
            elif(self.phi_ref - self.phi < -self.tolerance):
                dPhi = - self.omega
            else:
                dPhi = 0
        else:
            dPhi = self.omega
            
        if(self.v_ref  - self.v > self.tolerance):
            dV = self.acc
        elif(self.v_ref  - self.v < -self.tolerance):
            dV = -self.acc     
        else:
            dV = 0
            
        dt = t - self.lastUpdate

        self.v   += dV*dt
        self.phi += dPhi*dt
        
        self.lastUpdate = t
        
    def resetTime(self):
        self.lastUpdate = 0
        
class Bike(object):
    def __init__(self, length, theta = 0, x_0 = 0, y_0 = 0, tailDistance = 0.5):
        self.length = length # length of robot
        self.CoM = length/2 
        self.acc_cent = 0
        self.radius = 0
        
        self.orientation = theta
        self.x = x_0
        self.y = y_0
        
        self.tailDistance = tailDistance
        
        self.cruiseControl = CruiseControl(1, 1*pi/180)
        self.cruiseControl.setVelocity(0)
        self.cruiseControl.setPhi(0)
        
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

class Driver(object):
    def __init__(self, nextX, nextY, nextAngle, nextV):
        self.v_ref = None
        self.phi_ref = None
        
        self.v = None
        self.phi = None
        
        self.x_ref = None
        self.y_ref = None
        self.angle_ref = None
        
        self.nextX = nextX
        self.nextY = nextY 
        self.nextAngle = nextAngle
        self.nextV = nextV
        self.currGoal = 0
        self.heading = 0
        
        self.pathUpdateDistance = 10
        self.headingWindow = 0
        
        self.K_p_v = None
        self.K_i_v = None
        self.K_d_v = None
        self.error_v = None
        self.iError_v = None
        self.dError_v = None
        
        self.K_p_phi = None
        self.K_i_phi = None
        self.K_d_phi = None
        self.error_phi = None
        self.iError_phi = None
        self.dError_phi = None
        
        self.psi_yevr = 0
        
        self.lastUpdate = 0
        
    def initVelocityPIDController(self, K_p, K_i = 0, K_d = 0):
        self.K_p_v = K_p
        self.K_i_v = K_i
        self.K_d_v = K_d
        self.error_v = 0
        self.iError_v = 0
        self.dError_v = 0
        self.v = 0
        
    def initPhiPIDController(self, K_p, K_i = 0, K_d = 0):
        self.K_p_phi = K_p
        self.K_i_phi = K_i
        self.K_d_phi = K_d
        self.error_phi = 0
        self.iError_phi = 0
        self.dError_v = 0
        self.phi = 0
        
    def setSetpoint(self, v_ref, phi_ref):
        self.setVelocitySetpoint(v_ref)
        self.setPhiSetpoint(phi_ref)
            
    def setVelocitySetpoint(self, v_ref):
        self.v_ref = v_ref
        
    def setPhiSetpoint(self, phi_ref):
        self.phi_ref = phi_ref
        
    def updateError(self, vel, phi, t):
        delta_t = t - self.lastUpdate
        self.updateVError(vel, delta_t)
        self.updatePhiError(phi, delta_t)
        self.lastUpdate = t
        
    def updateVError(self, vel, delta_t):
        self.dError_v = (v - self.v)/delta_t
        self.error_v = self.v_ref - vel
        self.iError_v += self.error_v*delta_t/2
        
    def updatePhiError(self, phi, delta_t):
        self.dError_phi = (phi - self.phi)/delta_t
        self.error_phi = self.phi_ref - phi
        self.iError_phi += self.error_phi*delta_t/2
        
    def calculateVelocityAction(self, vel, t):
        self.updateVError(vel, t)
        return self.K_p_v*self.error_v + self.K_i_v*self.iError_v + self.K_d_v*self.dError_v
    
    def calculatePhiAction(self, phi, t):
        self.updatePhiError(phi, t)    
        return self.K_p_phi*self.error_phi + self.K_i_phi*self.iError_phi + self.K_d_phi*self.dError_phi

    def setRoadTracker(self, x_ref, y_ref, angle_ref, v_ref):
        self.x_ref = np.array(self.smooth(x_ref))
        self.y_ref = np.array(self.smooth(y_ref))
        self.angle_ref = np.array(self.smooth(angle_ref))
        self.v_ref = np.array(v_ref)
        t_ref = [0]
        t = 0
        for i in range(1,len(self.x_ref)):
            distance = sqrt((self.x_ref[i-1] - self.x_ref[i])**2 + (self.y_ref[i-1] - self.y_ref[i])**2)
            t  += distance/((self.v_ref[i-1] + self.v_ref[i])/2)
            t_ref.append(t)
        self.t_ref = np.array(t_ref)
        
    def smooth(self, path, weight_data = 0.5, weight_smooth = 0.2, tolerance = 0.000001):
        ''' Using the code for smooth function from the course the artificial inteligence 
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
        
    def getPath(self, x, y):
        idx = self.getPathIndex(x,y)            
        return (self.x_ref[idx], self.y_ref[idx], self.angle_ref[idx])     
    
    def getPathIndex(self, x, y):
        distance = (self.x_ref - x)**2 + (self.y_ref - y)**2
        idx = distance.argmin()
        self.currGoal = idx
        return idx
    
    def getNextGoal(self, x, y):
        lIdx = max(self.currGoal - self.headingWindow , 0)
        rIdx = min(self.currGoal + self.headingWindow , len(self.x_ref) - 1)
        self.nextX = (self.x_ref[lIdx] + self.x_ref[self.currGoal] + self.x_ref[rIdx])/3
        self.nextY = (self.y_ref[lIdx] + self.y_ref[self.currGoal] + self.y_ref[rIdx])/3
        self.nextAngle = (self.angle_ref[lIdx] + self.angle_ref[self.currGoal] + self.angle_ref[rIdx])/3

        distance = sqrt((self.nextX - x)**2 + (self.nextY - y)**2)
        if(distance < self.pathUpdateDistance):
            self.currGoal = min(self.currGoal + 1, len(self.x_ref) - 1)

        lIdx = max(self.currGoal - 1, 0)
        rIdx = min(self.currGoal + 1, len(self.x_ref) - 1)
        self.nextX = (self.x_ref[lIdx] + self.x_ref[self.currGoal] + self.x_ref[rIdx])/3
        self.nextY = (self.y_ref[lIdx] + self.y_ref[self.currGoal] + self.y_ref[rIdx])/3
        self.nextAngle = (self.angle_ref[lIdx] + self.angle_ref[self.currGoal] + self.angle_ref[rIdx])/3
#        self.nextAngle = atan2(self.y_ref[rIdx] - self.y_ref[lIdx], self.y_ref[rIdx] - self.y_ref[lIdx])
#        self.nextAngle = self.nextAngle % (2*pi)
#        if(self.nextAngle > pi):    
#            self.nextAngle -= 2*pi   
        self.nextV = self.v_ref[self.currGoal]
                        
        return [self.nextX, self.nextY, self.nextAngle, self.nextV]
        
    def calculateTracker(self, x, y, orientation, phi, delta_t):
        x_road, y_road, angle_road, _ = self.getNextGoal(x, y)
        eps = 0.1
        delta_x = (x_road - x) 
        delta_y = (y_road - y) 

        if(abs(delta_x) < eps) or (abs(delta_y) < eps):
            return 0
        elif (delta_x > 0):
            heading = atan(delta_y/delta_x)
        
        if (delta_x < 0):
            heading = atan(delta_y/delta_x) - pi

        heading = heading % (2*pi)
        if(heading > pi):    
            heading -= 2*pi                        

        heading -= orientation
        
        self.error_phi = (heading  - phi) % (2*pi)
#        self.error_phi = (heading  - orientation) % (2*pi)
        
        if(self.error_phi > pi):
            self.error_phi -= 2*pi
        
        self.iError_phi += self.error_phi*delta_t
        u = self.error_phi*self.K_p_phi + self.iError_phi*self.K_i_phi

        return u
    
    def calculateAdvancedTracker(self, x, y, orientation, phi, length, v, delta_t):

        x_road, y_road, angle_road, v_d = self.getNextGoal(x, y)
        eps = 0.1

        delta_x = (x_road - x) 
        delta_y = (y_road - y) 
        delta_o = (angle_road - orientation) % (2*pi)
        omega = sin(phi)/length

        if(delta_o  > pi):    
            delta_o -= 2*pi                        

        if(abs(delta_x) < eps) or (abs(delta_y) < eps):
            return 0
        else:
            heading = atan(delta_y/delta_x)
        
        if (delta_x < 0):
            heading = atan(delta_y/delta_x) - pi

        heading = heading % (2*pi)
        if(heading > pi):    
            heading -= 2*pi                        

        heading -= orientation
        dHeading = (heading - self.heading)/delta_t
        self.heading = heading
        
        self.error_phi = (heading  - phi) % (2*pi)
        
        if(self.error_phi > pi):
            self.error_phi -= 2*pi
        
        self.iError_phi += self.error_phi*delta_t
        
        dE_1 = v_d*cos(delta_o) + omega*delta_y - v*cos(phi)
        dE_2 = v_d*sin(delta_o) - omega*delta_x
        dPhi_d = (delta_x*dE_2 - delta_y*dE_1)/(delta_x**2 + delta_y**2) - omega
#        print(y_road)
#        print(x_road)
#        print()
#        print(dPhi_d)
#        print()
        u = 0
        u = self.error_phi*self.K_p_phi + dHeading
#        u = dHeading
#        u = self.error_phi*self.K_p_phi 

#        if(u>0):
#            u = min(u, pi/6)
#        else:
#            u = max(u, -pi/6)
            
        return u
    
    def cteLoop(self, x, y, delta_t):
        distance = (self.x_ref - x)**2 + (self.y_ref - y)**2
        idx = distance.argmin()
        
        aIdx = min(idx+1, len(self.x_ref) - 1)
        bIdx = max(idx-1, 0)
        
        if (idx == 0) or (distance[bIdx] >= distance[aIdx]):
            lIdx = idx
            rIdx = aIdx
        else:
            lIdx = bIdx
            rIdx = idx
            
        y_1 = self.y_ref[lIdx]
        y_2 = self.y_ref[rIdx]
        
        x_1 = self.x_ref[lIdx]
        x_2 = self.x_ref[rIdx]
        
        cte = abs((y_2 - y_1)*x - (x_2 - x_1)*y + x_2*y_1 - y_2*x_1)/sqrt((y_2-y_1)**2 + (x_2-x_1)**2)
        
        side = (x_1 - x)*(y_2 - y) - (x_2 - x)*(y_1 - y)
        if side > 0:
            self.dError_phi = (-cte - self.error_phi)/delta_t
            self.error_phi = -cte
        elif side < 0:
            self.dError_phi = (cte - self.error_phi)/delta_t
            self.error_phi = cte
        else:
            self.dError_phi  = 0
            self.error_phi = 0
            
        self.iError_phi += self.error_phi*delta_t

        u = self.error_phi*self.K_p_phi + self.iError_phi*self.K_i_phi + self.dError_phi*self.K_d_phi
        
        return u
    
    def backsteppingControl_unicyle(self, x, y, theta, dt):
        '''
        REf: Tracking Control of Mobile Robots: A Case Study in backstepping.
        
        After way longer than I'd like to admit I realized that the paper was not
        considering a bycicle model. Not working 
        
        JIANGdagger, Z.P. and Nijmeijer, H., 1997. Tracking control of mobile robots: a case study in backstepping. Automatica, 33(7), pp.1393-1399.
        '''
        currV = self.nextV
        currAngle = self.nextAngle % (2*pi)
        currAngle = currAngle  % (2*pi)
        theta = theta % (2*pi)
        
        x_r, y_r, theta_r, v_r = self.getNextGoal(x,y)
        theta_r = theta_r % (2*pi)
        dV_r = (v_r - currV)/dt
        omega_r = (theta_r - currAngle)/dt
#        omega_r = 0

        x_e = cos(theta)*(x_r - x) + sin(theta)*(y_r - y)
        y_e = -sin(theta)*(x_r - x) + cos(theta)*(y_r - y)
        theta_e = theta_r - theta
        
        
        psi_yevr = self.psi(y_e*v_r) 
        dPsi_yevr = (psi_yevr - self.psi_yevr)/dt
        self.psi_yevr = psi_yevr

        
        thetaBar_e = theta_e + psi_yevr
        nu = self.nuInt(psi_yevr, thetaBar_e)
        
        c_1 = .00001
        c_2 = .0001
        gamma = 0.01
            
        
        u_v = v_r*cos(theta_e) + c_1*x_e
        u_omega =  ((gamma*nu*y_e*v_r + omega_r + \
                 dPsi_yevr*(v_r*v_r*sin(theta_e) + y_e*dV_r) + c_2*gamma*thetaBar_e)/\
                 (1 + dPsi_yevr*x_e*v_r)).item(0)
#        
        u_v = max(v_r, u_v)    
        if(u_omega < 0):
            u_omega = max(-1, u_omega)
        elif(u_omega > 0):
            u_omega = min(1, u_omega)
#            
        print(u_omega)            
        return u_v, u_omega


    def backsteppingControl(self, x, y, theta, phi, length, dt):
        '''
        REf: Tracking Control of Mobile Robots: A Case Study in backstepping.
        
        After way longer than I'd like to admit I realized that the paper was not
        considering a bycicle model. Not working 
        
        JIANGdagger, Z.P. and Nijmeijer, H., 1997. Tracking control of mobile robots: a case study in backstepping. Automatica, 33(7), pp.1393-1399.
        '''
        currAngle = self.nextAngle 
        currAngle = currAngle 
        
        x_r, y_r, theta_r, v_r = self.getNextGoal(x,y)
        theta_r = theta_r

        omega_r = (theta_r - currAngle)/dt
        y_r = v_r*cos(theta_r)*dt + self.nextY
        x_r = v_r*sin(theta_r)*dt + self.nextX
        
        self.nextX = x_r
        self.nextY = y_r
#        omega_r = 0

        x_e = cos(theta)*(x_r - x) + sin(theta)*(y_r - y)
        y_e = -sin(theta)*(x_r - x) + cos(theta)*(y_r - y)
        theta_e = theta_r - theta
        
        a = .3 #natural frequency 
        zeta = 0.8 
        k1 = 2*zeta*a*cos(phi)
        k2 = (a*a - omega_r**2)/v_r/length
        k3 = 2*zeta*a/length
        
        u_1 = -k1*x_e
        u_2 = -k2*y_e - k3*theta_e
        
        print(x_e)
        u_v = (v_r*cos(theta_e) - u_1)/cos(phi)
        u_omega = asin((omega_r  - u_2tan)*length)
         
#        u_v = max(v_r, u_v)    
#        if(u_omega < 0):
#            u_omega = max(-1, u_omega)
#        elif(u_omega > 0):
#            u_omega = min(1, u_omega)
##            
#        print(u_omega)            
        return u_v, u_omega


    def psi(self, z):
        sigma = pi/6

        return sigma*z/(1 + z*z)
    
    def nuInt(self, psi_yevr, theta):
        def diffNu(X, s, psi_yevr, theta ):
            return cos(-psi_yevr + s*theta)
        
        returnArray = integrate.odeint(diffNu, [0], [0,1], args = (psi_yevr, theta))
        return returnArray[-1]
        
    def centripetalLoop(self, acc_cent, max_lat_accel, l_1, phi_1, v_1, dV, omega):
        max_v_dyn = sqrt(max_lat_accel*l_1/sin(phi_1))
#        dAcc_cet = abs(2*v_1*dV*cos(phi_1)*omega/l_1)
#        derivative_factor = .2
##        print(omega)
#        if(omega > 0):
#            omega -= derivative_factor*dAcc_cet
#            omega = max(0, omega)
#        else:
#            omega += derivative_factor*dAcc_cet
#            omega = min(0, omega)
##        print(omega)
#        
##        print()
        return max_v_dyn
        
class BikeTrailer(object):
    def __init__(self, theta = 0, x_0 = 0, y_0 = 0, lengthTractor = 4, lengthTrailer = 10, tailDistance = 0.5):
        self.tractor = Bike(lengthTractor, theta, x_0, y_0, tailDistance )

        self.trailer = Bike(lengthTrailer, 
                            theta, 
                            self.tractor.x - cos(self.tractor.orientation)*(self.tractor.length + self.tractor.tailDistance),
                            self.tractor.y - sin(self.tractor.orientation)*(self.tractor.length + self.tractor.tailDistance),
                            tailDistance)
        
        self.driver = Driver(self.tractor.x, self.tractor.y, self.tractor.orientation, self.tractor.cruiseControl.v)
        self.driver.initVelocityPIDController(K_p = 1, K_i = 0.2, K_d = 0)
#        self.driver.initPhiPIDController(K_p = 4, K_i = 0.5, K_d = 0)
#        self.driver.initPhiPIDController(K_p = 8, K_i = 0.4, K_d = 0)
        self.driver.initPhiPIDController(K_p = 0.1, K_i = 0, K_d = 0)
        
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
        self.tractor.cruiseControl.useCTELoop = False
        return self.driver.calculateTracker(self.tractor.x, self.tractor.y, self.tractor.orientation, 
                                               self.tractor.cruiseControl.phi, dt)  

    def calculateAdvancedTracker(self, dt):
        self.tractor.cruiseControl.useCTELoop = False
        return self.driver.calculateAdvancedTracker(self.tractor.x, self.tractor.y, self.tractor.orientation, 
                                               self.tractor.cruiseControl.phi, self.tractor.length, 
                                               self.tractor.cruiseControl.v, dt)  

        
    def cteLoop(self, dt):
        self.tractor.cruiseControl.useCTELoop = True
        return self.driver.cteLoop(self.tractor.x, self.tractor.y, dt)  
     
    def backsteppingControl(self, dt):
        self.tractor.cruiseControl.useCTELoop = False
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
        self.tractor.cruiseControl.resetTime()
    
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

        self.tractor.cruiseControl.updateStates(t)
        
        phi_1 = self.tractor.cruiseControl.phi
        v_1 = self.tractor.cruiseControl.v
        
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
#
#if __name__ == '__main__':
#    truck = BikeTrailer()
#    truck.createPlot()
#    truck.plotRefreshRate = 0.1
#    
#    totalTime = 10
#    
#    print('$$$$$ First Move $$$$$$')
#    truck.print()
#    print()
#    omega = 0.001
#    v = 10
#    truck.tractor.cruiseControl.resetTime()
#    truck.timeOfLastPlot = -truck.plotRefreshRate
#    truck.tractor.cruiseControl.setOmega(omega)
#    truck.tractor.cruiseControl.setVelocityRef(v)
##    truck.tractor.cruiseControl.setPhiRef(1)
#    truck.move(totalTime)
#    
#    print('$$$$$ Second Move $$$$$$')
#    truck.print()
#    print()
#    totalTime = 20
#    omega = 0.001
#    v = 20
#    truck.tractor.cruiseControl.resetTime()
#    truck.timeOfLastPlot = -truck.plotRefreshRate
#    truck.tractor.cruiseControl.setOmega(omega)
#    truck.tractor.cruiseControl.setVelocityRef(v)
#    truck.move(totalTime)
#    
#    print('$$$$$ Third Move $$$$$$')
#    truck.print()
#    print()
#    totalTime = 30
#    omega = 0.00
#    v = 10
#    truck.tractor.cruiseControl.resetTime()
#    truck.timeOfLastPlot = -truck.plotRefreshRate
#    truck.tractor.cruiseControl.setOmega(omega)
#    truck.tractor.cruiseControl.setVelocityRef(v)
#    truck.move(totalTime)
#    
#    
#    print('$$$$$ Fourth Move $$$$$$')
#    truck.print()
#    print()
#    totalTime = 30
#    omega = -0.002
#    v = 20
#    truck.tractor.cruiseControl.resetTime()
#    truck.timeOfLastPlot = -truck.plotRefreshRate
#    truck.tractor.cruiseControl.setOmega(omega)
#    truck.tractor.cruiseControl.setVelocityRef(v)
#    truck.move(totalTime)
#    
#    
#    print('$$$$$ Fifth Move $$$$$$')
#    truck.print()
#    print()
#    totalTime = 30
#    omega = 0
#    v = 10
#    truck.tractor.cruiseControl.resetTime()
#    truck.timeOfLastPlot = -truck.plotRefreshRate
#    truck.tractor.cruiseControl.setOmega(omega)
#    truck.tractor.cruiseControl.setVelocityRef(v)
#    truck.move(totalTime)