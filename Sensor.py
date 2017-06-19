# -*- coding: utf-8 -*-
"""
Created on Sun Jun 18 20:25:02 2017

@author: leo
"""

class Sensor(object):
    '''
    The sensor class is responsible to acquire data from the enviroment
    '''
    
    def __init__(self):
        self.measure = None
        self.scanner = None
        
    def connect(self, scanner):
        self.scanner = scanner
        
    def scan(self):
        self.measure = self.scanner()
        
    def read(self):
        return self.measure