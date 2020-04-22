#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 21 16:51:34 2020

@author: osboxes
"""

#move2
import tinyik
from joy.plans import Plan
from joy import progress
from numpy import linspace,dot,zeros,pi,rad2deg,asarray,meshgrid,ones,c_
from numpy.linalg import pinv,inv
from scipy.interpolate import griddata

#armSpec = asarray([
#        [0,0.02,1,5,0],
#        [0,1,0,5,1.57],
#        [0,1,0,5,0],
#      ]).T

class Move( Plan ):
    def __init__(self,app,*arg,**kw):
        Plan.__init__(self,app,*arg,**kw)
        self.app = app
        #Initial angles for each motor of real arm
        self.steps = []
        self.moveArm = tinyik.Actuator(['z',[5.,0.,0.],'y',[5.,0.,0.],'y',[5.,0.,0.]])
        self.moveArm.angles = self.app.armSpec[-1,:]
        self.pos = []
        self.calibrated = False
    
    #used to get initial joint angles before autonomous move
    def syncArm(self):
        ang = zeros(len(self.app.arm))
        for i,motor in enumerate(self.app.arm):
            ang[i] = motor.get_goal()*(3.14159/18000.)   #convert angles to radians
        self.moveArm.angles = ang
    
    #for moving towwards desired position
    def behavior(self):
        angOffset = zeros(len(self.app.arm))
        if self.calibrated == True:
            #Interpolate between angle grid positions and angle error
            progress('Offset applied')
            angOffset = griddata(self.app.calib_grid[:,:-2],self.app.calib_ang,(self.pos[:-2]),method='linear')[0]
        print(angOffset)

        self.syncArm()       
        currentPos = self.app.idealArm.getTool(self.moveArm.angles)
        self.steps = linspace(currentPos,self.pos,5)[:,:-1]
        for stepCount,step in enumerate(self.steps):
            progress('Step #%d, %s' % (stepCount,str(step)))
            self.app.currStep = stepCount
            self.moveArm.ee = step
            for i,motor in enumerate(self.app.arm):
                motor.set_pos(rad2deg(self.moveArm.angles[i]+angOffset[i])*100)    #feed in angle to set_pos as centidegrees
            yield self.forDuration(4)
        progress('Move complete')