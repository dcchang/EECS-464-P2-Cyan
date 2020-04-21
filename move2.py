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
from numpy import linspace,dot,zeros,pi,rad2deg
from numpy.linalg import pinv,inv

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
        self.ang = self.app.armSpec[-1,:]
        self.steps = []
        self.moveArm = tinyik.Actuator(['z',[5.,0.,0.],'y',[5.,0.,0.],'y',[5.,0.,0.]])
        self.moveArm.angles = [0,pi/2,0]
        self.curr = 0

    def syncArm(self):
        self.ang = zeros(len(self.app.arm))
        for i,motor in enumerate(self.app.arm):
            self.ang[i] = motor.get_goal()*(3.14159/18000.)   #convert angles to radians

    def behavior(self):
        progress('Move started!')
        self.syncArm()
        corner = self.app.square_w[self.curr]
        currentPos = self.app.idealArm.getTool(self.ang)
        self.steps = linspace(currentPos,corner,5)[:,:-1]

        #Converts steps from world to workspace
#        self.steps = dot(self.steps,self.app.Tw2ws.T)[:,:-1]

        for stepCount,step in enumerate(self.steps):
            progress('Step #%d, %s' % (stepCount,str(step)))
            self.app.currStep = stepCount
            self.moveArm.ee = step
            for i,motor in enumerate(self.app.arm):
                motor.set_pos(rad2deg(self.moveArm.angles[i])*100)    #feed in angle to set_pos as centidegrees
            yield self.forDuration(5)
        progress('Move complete')
        self.curr += 1
