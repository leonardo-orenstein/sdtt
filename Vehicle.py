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
        
    def calculateAdvancedTracker(self, dt):
        x, y        = self.gps.read()
        orientation = self.compass.read()
        phi         = self.wheelAngle.read()
        v           = self.speedometer.read()
        x_road, y_road, angle_road, v_d = self.planner.getGoal(x, y)

        return self.pilot.headingTracker(x, y, orientation, 
                                           phi, v, self.length,
                                           x_road, y_road, angle_road, v_d,
                                           dt)  

    def velocityLoop(self, v_ref, acc):
        v           = self.speedometer.read()
        return self.pilot.velocityLoop(v, v_ref, acc, histeresis = 0.1)


    def connectToEngine(self, function):
        self.engine.connect(function)
        
    def connectToSteering(self, function):
        self.steering.connect(function)    
        
    def connectToSpeedometer(self, function):
        self.speedometer.connect(function)
        
    def connectToWheelAngle(self, function):
        self.wheelAngle.connect(function)
        
    def connectToGPS(self, function):
        self.gps.connect(function)
        
    def connectToCompass(self, function):
        self.compass.connect(function)
        
    def connectToSimulation(self, system):
        self.connectToEngine(system.setAcceleration)
        self.connectToSteering(system.setOmega)
        
        self.connectToCompass(system.getOrientation)
        self.connectToSpeedometer(system.getVelocity)
        self.connectToGPS(system.getPosition)
        self.connectToWheelAngle(system.getSteering)
        
        
    def scan(self):
        self.speedometer.scan()
        self.compass.scan()
        self.gps.scan()
        self.wheelAngle.scan()
        
    def plot(self):
        x, y        = self.gps.read()
        orientation = self.compass.read()
        self.vizualizer.plot(x, y, orientation, self.length)
        
        
    def plot3d(self):
        x, y        = self.gps.read()
        orientation = self.compass.read()
        self.vizualizer.plot3d(x, y, orientation)