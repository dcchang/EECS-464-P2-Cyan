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
        self.ang = self.app.armSpec[-1,:]
        self.steps = []

    def syncArm(self):
        self.ang = zeros(len(self.app.arm))
        for i,motor in enumerate(self.app.arm):
            self.ang[i] = motor.get_goal()*(3.14159/18000.)   #convert angles to radians
    def behavior(self):
        progress('Move started!')
        self.syncArm()
        corner = self.app.square_w[0]
        currentPos = self.app.idealArm.getTool(self.ang)
        self.steps = linspace(currentPos,corner,15)[:,:-1]
        for stepCount,step in enumerate(self.steps[:-1]):
            progress('Step #%d, %s' % (stepCount,str(step)))
            self.app.currStep = stepCount + 1
            Jt = self.app.idealArm.getToolJac(self.ang)     #takes in angle as radian
            d = self.steps[stepCount+1] - step     #distance betwen current position and next position
            
            angDiff = dot(pinv(Jt)[:,:len(d)],d)
            progress('Angle Diff 1: %s'% str(angDiff*180/3.14159))    #show angle diff in degrees
            for i,angle in enumerate(angDiff):
                progress('its happening')
                print(angle)
                if angle > 3.1415 :
                    print('angDiff1: ', angDiff[i])
                    angDiff[i] = angDiff[i] - 2*3.1415 
                    print('angDiff2: ', angDiff[i])
                elif angle < -3.1415:
                    print('angDiff1: ', angDiff[i])
                    angDiff[i] = angDiff[i] + 2*3.1415 
                    print('angDiff2: ', angDiff[i])
            progress('Angle Diff 2: %s'% str(angDiff*180/3.14159))    #show angle diff in degrees
            self.ang += angDiff
            for i,motor in enumerate(self.app.arm):
                motor.set_pos(self.ang[i]*18000./3.14159)    #feed in angle to set_pos as centidegrees
#           
#            progress('Step #%d, %s' % (stepCount,str(step)))
#            progress('Distance %s' % (str(d)))
#            progress('Angle Diff: %s'% str(angDiff*180/3.14159))    #show angle diff in degrees
#            progress('Angles: %s'% str(self.ang*180/3.14159))       #show angles in degrees
            yield self.forDuration(7)
        progress('Move complete')
        
