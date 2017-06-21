# -*- coding: utf-8 -*-
"""
Created on Sun Jun 18 20:22:33 2017

@author: leo
"""

from Actuator import Actuator
from Sensor import Sensor
from Pilot import Pilot
from Planner import Planner
from Vizualizer import Vizualizer
from ParticleFilter import ParticleFilter
from SimulationVehicle import Bike

class Vehicle(object):
    '''
    The overal Vehicle class pull together all other components
    '''
    def __init__(self, length, fig, ax):
        self.engine = Actuator()
        self.steering = Actuator()
        
        self.speedometer = Sensor()
        self.wheelAngle = Sensor()
        self.gps = Sensor()
        self.compass = Sensor()
        
        self.pilot = Pilot()
        self.planner = Planner()
        
        self.vizualizer = Vizualizer(length, fig, ax)  
        self.length = length
        
        self.maxLonAcc = 10
        self.particleFilter = None
        
    def headingTracker(self, dt):
        '''
        Wrapper for the heading controller
        '''
#        x, y        = self.gps.read()
        x, y        = self.particleFilter.getPosition()
        orientation = self.compass.read()
        phi         = self.wheelAngle.read()
        v           = self.speedometer.read()
        x_road, y_road, angle_road, v_d = self.planner.getGoal(x, y, v, dt)

        return self.pilot.headingTracker(x, y, orientation, 
                                           phi, v, self.length,
                                           x_road, y_road, angle_road, v_d,
                                           dt)  

    def phiController(self, dt):
        '''
        Wrapper for the phi controller
        '''
#        x, y        = self.gps.read()
        x, y        = self.particleFilter.getPosition()        
        orientation = self.compass.read()
        phi         = self.wheelAngle.read()
        v           = self.speedometer.read()
        x_road, y_road, angle_road, v_d = self.planner.getGoal(x, y, v, dt)

        return self.pilot.phiController(x, y, orientation, phi,
                                        x_road, y_road, angle_road,
                                       dt)  

    def cteController(self, dt):
        '''
        Wrapper for the Cross Track Error controller
        '''
#        x, y        = self.gps.read()    
        x, y        = self.particleFilter.getPosition()
        return self.pilot.crossTrackErrorController(x, y, self.planner, dt)
    
    def velocityController(self, dt):
        '''
        Wrapper for the velocity controller
        '''
        x, y        = self.gps.read()
        x, y        = self.particleFilter.getPosition()
        v               = self.speedometer.read()
        _, _, _, v_ref  = self.planner.getGoal(x, y, v, dt)

        return self.pilot.velocityController(v, v_ref, self.maxLonAcc, histeresis = 0.1)


    def connectToEngine(self, function):
        '''
        Adds a connection to the engine
        '''
        self.engine.connect(function)
        
    def connectToSteering(self, function):
        '''
        Adds a connection to the steering wheel
        '''
        self.steering.connect(function)    
        
    def connectToSpeedometer(self, function):
        '''
        Adds a connection to the velocimeter
        '''
        self.speedometer.connect(function)
        
    def connectToWheelAngle(self, function):
        '''
        Adds a connection to system to measure the steering wheel angle
        '''
        self.wheelAngle.connect(function)
        
    def connectToGPS(self, function):
        '''
        Adds a connection to system that captures the vehicle position
        '''
        self.gps.connect(function)
        
    def connectToCompass(self, function):
        '''
        Adds a connection to system that captures the vehicle orientation
        '''
        self.compass.connect(function)
        
    def connectToSimulation(self, system):
        '''
        Wrapper to connect all system to simulation model
        '''
        self.connectToEngine(system.setAcceleration)
        self.connectToSteering(system.setOmega)
        
        self.connectToCompass(system.getOrientation)
        self.connectToSpeedometer(system.getVelocity)
        self.connectToGPS(system.getPosition)
        self.connectToWheelAngle(system.getSteering)
        
        
    def scan(self):
        '''
        Do a scan round through all of the sensors    
        '''
        self.speedometer.scan()
        self.compass.scan()
        self.gps.scan()
        self.wheelAngle.scan()
    
    def createFilter(self):
        xHat, yHat      = self.gps.read()
        orientationHat  = self.compass.read()
        phiHat          = self.wheelAngle.read()
        vHat            = self.speedometer.read()
        self.particleFilter = ParticleFilter(Bike, xHat, yHat, orientationHat, phiHat, vHat, self.length)
        
    def updateFilter(self, dt):
        xHat, yHat      = self.gps.read()
        orientationHat  = self.compass.read()
        phiHat          = self.wheelAngle.read()
        vHat            = self.speedometer.read()
        acc             = self.engine.getValue()
        omega           = self.steering.getValue()
        
#        measurements = [ xHat, yHat, orientationHat, phiHat, vHat]
        measurements = [ xHat, yHat]
        self.particleFilter.addData(acc, omega, dt, measurements)

        
    def plot(self):
        '''
        Wrapper for the vizualizer to update the plot
        '''
        x, y        = self.gps.read()
        orientation = self.compass.read()
        self.vizualizer.plot(x, y, orientation, self.length)
        
        
    def plot3d(self):
        '''
        Wrapper for the 3d plotting of the vizualizer. Not yet implement
        '''
        x, y        = self.gps.read()
        orientation = self.compass.read()
        self.vizualizer.plot3d(x, y, orientation)