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
        self.steps = linspace(currentPos,corner,10)[:,:-1]
        for stepCount,step in enumerate(self.steps[1:]):
            self.app.currStep = stepCount + 1
            Jt = self.app.idealArm.getToolJac(self.ang)     #takes in angle as radian
            d = step - self.steps[stepCount]    #distance betwen current position and next position
            angDiff = dot(pinv(Jt)[:,:-1],d)
            self.ang += angDiff
            for i,motor in enumerate(self.app.arm):
                motor.set_pos(self.ang[i]*18000./3.14159)    #feed in angle to set_pos as centidegrees
            progress('Step #%d, %s' % (stepCount,str(step)))
            progress('Distance %s' % (str(d)))
            progress('Angle Diff: %s'% str(angDiff))
            progress('Angles: %s'% str(self.ang))
            yield self.forDuration(5)
        progress('Move complete')
