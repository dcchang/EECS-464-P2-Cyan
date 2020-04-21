#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 21 03:09:58 2020

@author: osboxes
"""
from joy.plans import Plan
from joy import progress
from numpy import linspace,dot,zeros
from numpy.linalg import pinv


class Move( Plan ):
    def __init__(self,app,*arg,**kw):
        Plan.__init__(self,app,*arg,**kw)
        self.app = app
        #Initial angles for each motor of real arm
        self.ang = zeros(len(self.app.arm))
        for i,motor in enumerate(self.app.arm):
            self.ang[i] = motor.get_pos()/100*(3.1415/180)   #convert angles to radians  
        self.steps = []
    def behavior(self):
        progress('Move started!')        
        corner = self.app.square_w[0]
        currentPos = self.app.idealArm.getTool(self.ang)
#      steps = currentPos + linspace(0,1,10)[...,newaxis]*(corner-currentPos)
        self.steps = linspace(currentPos,corner,20)
        self.steps = self.steps[:,:-1]
        for stepCount,step in enumerate(self.steps[1:]):
            progress('Step %d\n%s' % (stepCount,str(step)))
            Jt = self.app.idealArm.getToolJac(self.ang)     #takes in angle as radian
            progress('Jacobian: %s' % str(Jt))
            d = step - self.steps[stepCount]    #distance betwen current position and next position
            angDiff = dot(pinv(Jt)[:,:len(d)],d)
#            if any(angDiff < 0):
#                for i,angle in enumerate(angDiff):
#                    if angle < 0:
#                        angDiff[i] += 2*3.1415
            self.ang += angDiff
            progress('Angles: %s'% str(self.ang))
            for i,motor in enumerate(self.app.arm):
                motor.set_pos(self.ang[i]*100*180/3.1415)    #feed in angle to set_pos as centidegrees
            yield self.forDuration(3)
            
        progress('Move complete')
            
          
        
        