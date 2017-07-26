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
from Body import Body

class Vehicle(Body):
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
        self.laneTracker = Sensor()

        self.pilot = Pilot()
        self.planner = Planner()

        self.vizualizer = Vizualizer(length, fig, ax)
        self.length = length

        self.maxLonAcc = 10
        self.particleFilter = None


    def getPos(self):
        return self.particleFilter.getPosition()

    def getOrientation(self):
        return self.compass.read()

    def getVelocity(self):
        return self.speedometer.read()

    def getAcc(self):
        return self.engine.getValue()

    def getOmega(self):
        return self.steering.getValue()

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

    def lqrTracker(self, dt):
        '''
        Wrapper for the heading controller
        '''
        x, y        = self.gps.read()
        # The particle filter is currently not working with the LQT as it's
        # using omega, instead of phi, as an input.
#        x, y        = self.particleFilter.getPosition()
        orientation = self.compass.read()
        phi         = self.wheelAngle.read()
        v           = self.speedometer.read()
        x_road, y_road, angle_road, v_d = self.planner.getPath(x[0], y[0])
#        x_road, y_road, angle_road, v_d = self.planner.getGoal(x, y, v, dt)

        return self.pilot.lqrTracker(x[0], y[0], orientation,
                                           phi, v, self.length, self.planner,
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

    def connectToLaneTracker(self, function):
        '''
        Adds a connection to system that captures the vehicle orientation
        '''
        self.laneTracker.connect(function)

    def connectToSimulation(self, simulationVehicle, laneTracker):
        '''
        Wrapper to connect all system to simulation model
        '''
        self.connectToEngine(simulationVehicle.setAcceleration)
        self.connectToSteering(simulationVehicle.setOmega)

        self.connectToCompass(simulationVehicle.getOrientation)
        self.connectToSpeedometer(simulationVehicle.getVelocity)
        self.connectToGPS(simulationVehicle.getPosition)
        self.connectToWheelAngle(simulationVehicle.getSteering)
        self.connectToLaneTracker(laneTracker)

    def scan(self):
        '''
        Do a scan round through all of the sensors
        '''
        self.speedometer.scan()
        self.compass.scan()
        self.gps.scan()
        self.wheelAngle.scan()
        self.laneTracker.scan()

    def createFilter(self):
        xHat, yHat      = self.gps.read()
        orientationHat  = self.compass.read()
        phiHat          = self.wheelAngle.read()
        vHat            = self.speedometer.read()
        self.particleFilter = ParticleFilter(Bike, xHat, yHat, orientationHat, phiHat, vHat, self.length,  200)

    def updateFilter(self, dt):
        xHat, yHat      = self.gps.read()
        xLane, yLane    = self.laneTracker.read()

        orientationHat  = self.compass.read()
        phiHat          = self.wheelAngle.read()
        vHat            = self.speedometer.read()
        acc             = self.engine.getValue()
        omega           = self.steering.getValue()

#        measurements = [ xHat, yHat, orientationHat, phiHat, vHat]
#        V = [self.gps.getUncertanty(), self.gps.getUncertanty(), self.compass.getUncertanty(), self.wheelAngle.getUncertanty(), self.speedometer.getUncertanty()]

        self.particleFilter.updateParticles(acc, omega, dt)

        measurements = [ xHat, yHat]
        V = [self.gps.getUncertanty(), self.gps.getUncertanty()]
        self.particleFilter.weightParticles(measurements, V)
        self.particleFilter.resampleParticles()

#        measurements = [xLane, yLane]
#        V = [self.laneTracker.getUncertanty(), self.laneTracker.getUncertanty()]
#        self.particleFilter.weightParticles(measurements, V)
#        self.particleFilter.resampleParticles()

    def plot(self, draw = True):
        '''
        Wrapper for the vizualizer to update the plot
        '''
        x, y        = self.gps.read()
        orientation = self.compass.read()
#        x_goal, y_goal = self.planner.nextX, self.planner.nextY
        self.vizualizer.plot(x, y, orientation, self.length, draw)


    def plot3d(self):
        '''
        Wrapper for the 3d plotting of the vizualizer. Not yet implement
        '''
        x, y        = self.gps.read()
        orientation = self.compass.read()
        self.vizualizer.plot3d(x, y, orientation)