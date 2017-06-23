# -*- coding: utf-8 -*-
"""
Created on Mon Jun 19 14:40:32 2017

@author: leo
"""
import matplotlib.pyplot as plt
from math import sin, cos

class Vizualizer(object):
    '''
    Vizualizer class: Creates vizualizations for the vehicle
    '''
    
    def __init__(self, length,
                         fig = None, ax = None, plotRefreshRate = 2):
        
        # handling plot
        self.plotRefreshRate = plotRefreshRate  # seconds
        self.timeOfLastPlot = -self.plotRefreshRate
        
        if(fig is None):
            self.fig = plt.figure()
        else:
            self.fig = fig
            
        if(ax is None):        
            self.ax = self.fig.add_subplot(111)
        else:
            self.ax = ax
                    
        self.front, = self.ax.plot(0,  0, 'r*')
        self.body, = self.ax.plot([0, 0 - cos(0)*length], [0, 0 - sin(0)*length], 'k')
        
#        self.body3d = box(pos=vector(0,0,0), axis=vector(0, 0, 0), length=length, height=2, width=2)

        self.ax.set_xlim([-50, 50])
        self.ax.set_ylim([-50, 50])
        self.ax.set_xlabel('Distance X')
        self.ax.set_ylabel('Distance Y')
        
    def plot(self, x, y, orientation, length, draw = True):
        self.front.set_ydata(y)
        self.front.set_xdata(x)
        self.body.set_ydata([y, y - sin(orientation)*length])
        self.body.set_xdata([x, x - cos(orientation)*length])
        
        self.ax.set_xlim([x - 50, x + 50])
        self.ax.set_ylim([y - 50, y + 50])
        
        if(draw):
            self.fig.canvas.draw()
            plt.pause(0.001)    
            
    def plot3d(self, x, y, orientation):
        self.body3d.pos  = vector(x,y,0)
        self.body3d.axis = vector(cos(orientation), sin(orientation), 0)

        
class VizualizerBikeTrailer(object):
    '''
    Vizualizer class: Creates vizualizations for the vehicle
    '''
    
    def __init__(self, tractor_x, tractor_y, trailer_x, trailer_y, trailer_orientation, trailerLength,
                         fig = None, ax = None, plotRefreshRate = 2):
        
        # handling plot
        self.plotRefreshRate = plotRefreshRate  # seconds
        self.timeOfLastPlot = -self.plotRefreshRate
        
        
        if(fig is None):
            self.fig = plt.figure()
        else:
            self.fig = fig
            
        if(ax is None):        
            self.ax = self.fig.add_subplot(111)
        else:
            self.ax = ax
                    
        self.tractorFront, = self.ax.plot(tractor_x, tractor_y, 'r*')
        self.trailerFront, = self.ax.plot(trailer_x, trailer_y, 'm*')
        self.tractorLine, = self.ax.plot([tractor_x, trailer_x], [tractor_y, trailer_y], 'k')
        self.trailerLine, = self.ax.plot([trailer_x, trailer_x - cos(trailer_orientation)*trailerLength], [trailer_y, trailer_y - sin(trailer_orientation)*trailerLength], 'b')
        self.ax.set_xlim([-50, 50])
        self.ax.set_ylim([-50, 50])
        self.ax.set_xlabel('Distance X')
        self.ax.set_ylabel('Distance Y')
        
    def plot(self, tractor_x, tractor_y, trailer_x, trailer_y, trailer_orientation, trailerLength):
        self.tractorFront.set_ydata(tractor_y)
        self.tractorFront.set_xdata(tractor_x)
        self.tractorLine.set_ydata([tractor_y, trailer_y])
        self.tractorLine.set_xdata([tractor_x, trailer_x])

        self.trailerFront.set_ydata(trailer_y)
        self.trailerFront.set_xdata(trailer_x)
        
        x_4 = trailer_x - (trailerLength)*cos(trailer_orientation)
        y_4 = trailer_y - (trailerLength)*sin(trailer_orientation)
        self.trailerLine.set_ydata([y_4, trailer_y])
        self.trailerLine.set_xdata([x_4, trailer_x])
        
        self.ax.set_xlim([trailer_x - 50, trailer_x + 50])
        self.ax.set_ylim([trailer_y - 50, trailer_y + 50])
        
        if(draw):
            self.fig.canvas.draw()
#            plt.pause(0.001)   
        
            
