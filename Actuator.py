# -*- coding: utf-8 -*-
"""
Created on Sun Jun 18 20:25:36 2017

@author: leo
"""

class Actuator(object):
    '''
    The actuator class turns the control action from the Pilot into actual
    physical action
    '''
    
    def __init__(self):
        self.system = None #Link with the physical system
        
    def connect(self, system):
        self.system = system #This is the connection with the the physical system. Should be a driver or something alike
        
    def setValue(self, value):
        self.system(value)
        