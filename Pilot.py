# -*- coding: utf-8 -*-
"""
Created on Sun Jun 18 20:23:56 2017

@author: leo
"""
from math import sin, cos, pi, sqrt, atan, asin
import scipy.integrate as integrate

class Pilot(object):
    '''
    The Pilot class has the controller used to follow a given path
    
    '''
    def __init__(self, nextX = None, nextY = None, nextAngle = None, nextV = None):        
        self.v = None
        self.phi = None
        
        self.v_ref = None
        self.phi_ref = None
        
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
        
        self.lastUpdate = 0
        
    def initVelocityPIDController(self, K_p, K_i = 0, K_d = 0, v = 0):
        self.K_p_v = K_p
        self.K_i_v = K_i
        self.K_d_v = K_d
        self.error_v = 0
        self.iError_v = 0
        self.dError_v = 0
        self.v = v
        
    def initPhiPIDController(self, K_p, K_i = 0, K_d = 0, phi = 0):
        self.K_p_phi = K_p
        self.K_i_phi = K_i
        self.K_d_phi = K_d
        self.error_phi = 0
        self.iError_phi = 0
        self.dError_v = 0
        self.phi = phi
        
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
        
    def updateVError(self, v, delta_t):
        self.dError_v = (self.v_ref - v)/delta_t
        self.error_v = self.v_ref - v
        self.iError_v += self.error_v*delta_t/2
        
    def updatePhiError(self, phi, delta_t):
        self.dError_phi = (self.phi_ref - self.phi)/delta_t
        self.error_phi = (self.phi_ref - self.phi) % (2*pi)
        if(self.error_phi > pi):
            self.error_phi -= 2*pi
        self.iError_phi += self.error_phi*delta_t/2

        
    def phiController(self, x, y, orientation, phi, 
                            x_road, y_road, angle_road,
                            delta_t):
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
        

        
        self.iError_phi += self.error_phi*delta_t
        u = self.error_phi*self.K_p_phi + self.iError_phi*self.K_i_phi

        return u
    
    def headingTracker(self, x, y, orientation, 
                           phi, v, length,  
                           x_road, y_road, angle_road, v_d,
                           delta_t):

        
        eps = 0.1

        delta_x = (x_road - x) 
        delta_y = (y_road - y) 
        delta_o = (angle_road - orientation) % (2*pi)

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
        
#        omega = sin(phi)/length
#        dE_1 = v_d*cos(delta_o) + omega*delta_y - v*cos(phi)
#        dE_2 = v_d*sin(delta_o) - omega*delta_x
#        dPhi_d = (delta_x*dE_2 - delta_y*dE_1)/(delta_x**2 + delta_y**2) - omega

        u = self.error_phi*self.K_p_phi + dHeading
            
        return u
    
    def crossTrackErrorController(self, x, y, planner, delta_t):
        distance = (planner.x_track - x)**2 + (planner.y_track - y)**2
        idx = distance.argmin()
        
        aIdx = min(idx+1, len(planner.x_track) - 1)
        bIdx = max(idx-1, 0)
        
        if (idx == 0) or (distance[bIdx] >= distance[aIdx]):
            lIdx = idx
            rIdx = aIdx
        else:
            lIdx = bIdx
            rIdx = idx
            
        y_1 = planner.y_track[lIdx]
        y_2 = planner.y_track[rIdx]
        
        x_1 = planner.x_track[lIdx]
        x_2 = planner.x_track[rIdx]
        
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
        
        u_v = (v_r*cos(theta_e) - u_1)/cos(phi)
        u_omega = asin((omega_r  - u_2)*length)
                   
        return u_v, u_omega

    def velocityLoop(self, v, v_ref, acc, histeresis = 0.1):
        if(abs(v - v_ref) <= histeresis):
            return 0
        elif(v < v_ref):
            return acc
        elif(v > v_ref):
            return -acc
        
        
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