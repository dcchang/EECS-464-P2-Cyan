#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 14:23:10 2020

@author: shrevzen
"""

#Added so you can run script from anywhere
import sys
import os
if 'pyckbot/hrb/' not in sys.path:
    sys.path.append(os.path.expanduser('~/pyckbot/hrb/'))

from numpy import asarray, dot
from numpy.linalg import inv
from p2sim import ArmAnimatorApp
from arm import Arm
from joy.decl import *
from joy import progress
from move2 import Move


class MyArmSim(ArmAnimatorApp):
    def __init__(self,Tp2ws,**kw):
      ###
      ### Student team selection -- transform from workspace coordinates to world
      ###
      Tws2w = asarray([
           [1,0,0, -6],
           [0,1,0, -5],
           [0,0,1,-10],
           [0,0,0,  1]
      ])
      ###
      ### Arm specification
      ###
      armSpec = asarray([
        [0,0.02,1,5,3.14159],
        [0,1,0,5,1.57],
        [0,1,0,5,0],
      ]).T

        #arm he gave that removes pendulum motion
#      armSpec = asarray([
#        [0, 1,0, 3, 1.57],
#        [0, 1,0, 3, 0],
#        [0, 1,0, 3, 0],
#        ]).T
      self.Tws2w = Tws2w
      self.Tw2ws = inv(self.Tws2w)
      self.armSpec = armSpec
      ArmAnimatorApp.__init__(self,armSpec,Tws2w,Tp2ws,
        simTimeStep=0.25, # Real time that corresponds to simulation time of 0.1 sec
        **kw
      )
      self.idealArm = Arm(armSpec)  #create instance of arm class without real-life properties
      self.move = Move(self)     #move plan

    def show(self,fvp):
      fvp.plot3D([0],[0],[0],'^k',ms=10) # Plot black triangle at origin
      for point in self.square_w:
          fvp.plot3D(*point[:-1],marker='o',color='r')

      for i,point in enumerate(self.move.steps):
          if self.currStep and self.currStep == i:
            fvp.plot3D(*point,marker='o',color='b')
          else:
            fvp.plot3D(*point,marker='o',color='g')

      return ArmAnimatorApp.show(self,fvp)

    def onStart(self):
      ArmAnimatorApp.onStart(self)
#      Tp2w:
#      array([[  0.7071,   0.    ,  -0.7071,   0.    ],
#       [  0.    ,   1.    ,   0.    ,  -5.    ],
#       [  0.7071,   0.    ,   0.7071, -10.    ],
#       [  0.    ,   0.    ,   0.    ,   1.    ]])
#
      ###
      ### TEAM CODE GOES HERE
      ###

      #Step 1: Get center of square in paper coordinates and convert to world coordinates
      #(x,y,s) - x target, y target, scale
      #Do you have to define a z component for the square? And add a column of 1's?
      x,y,s = [5,5,2]    #Prof will give this to us
      #Define 4 corners in paper coordinates
      square_p = asarray([
                  [x-0.5*s, y+0.5*s, 0, 1],     #upper left
                  [x+0.5*s, y+0.5*s, 0, 1],     #uper right
                  [x+0.5*s, y-0.5*s, 0, 1],     #bottom right
                  [x-0.5*s, y-0.5*s, 0, 1]      #bottom left
                  ])

      #Convert all coordinates for square to world coordinates
      self.square_w = dot(square_p, self.Tp2w.T)

    def onEvent(self,evt):
      ###
      ### TEAM CODE GOES HERE
      ###    Handle events as you see fit, and return after

      if evt.type == KEYDOWN:
          progress('in keydown')
          if evt.key == K_m:
              if self.move.isRunning():
                  progress('Move running!')
                  return
              progress('Move plan started!')
              self.move.start()


      return ArmAnimatorApp.onEvent(self,evt)

if __name__=="__main__":
  # Transform of paper coordinates to workspace
  Tp2ws = asarray([
       [0.7071,0,-0.7071,0],
       [0,     1,      0,0],
       [0.7071,0, 0.7071,0],
       [0,     0,      0,1]
  ])
  app = MyArmSim(Tp2ws,
     ## Uncomment the next line (cfg=...) to save video frames;
     ## you can use the frameViewer.py program to view those
     ## frames in real time (they will not display locally)
     # cfg=dict(logVideo="f%04d.png")
    )
  app.run()
