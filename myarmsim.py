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

from numpy import linspace,dot,zeros,pi,rad2deg,asarray,meshgrid,ones,c_,save,load
from numpy.linalg import inv 
from p2sim import ArmAnimatorApp
from arm import Arm
from joy.decl import KEYDOWN,K_k,K_o
from joy import progress
from move import Move

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
        [0,0.02,1,3,-pi],
        [0,1,0,4,1.57],
        [0,1,0,5,0],
      ]).T

      #arm he gave that removes pendulum motion
#      armSpec = asarray([
#        [0, 1,0, 3, 1.57],
#        [0, 1,0, 3, 0],
#        [0, 1,0, 3, 0],
#      ]).T
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
     
      for point in self.calib_grid:
          fvp.plot3D(*point[:-1],marker='o',color='m')
      return ArmAnimatorApp.show(self,fvp)

    def onStart(self):
      ArmAnimatorApp.onStart(self)

      ###
      ### TEAM CODE GOES HERE
      ###

      #Get center of square in paper coordinates and convert to world coordinates
      #(x,y,s) - x target, y target, scale
      
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
      
      ##Calibration
      
      #if calibration file exists, load calibration array in here, and skip over next part
      #also set calibrated == true so that it calculates offset
            #Create calibration grid on paper
      nx,ny = (2,2)
      x = linspace(0,8,nx)
      y = linspace(0,11,ny)
      xv,yv = meshgrid(x,y,indexing='xy')
      grid = c_[xv.reshape(nx*ny,1), yv.reshape(nx*ny,1), zeros((nx*ny,1)), ones((nx*ny,1))]
      grid_idx = list(range(nx*ny))
      for i in range(nx-(nx % 2)):
          idx = nx*(2*i+1)
          grid_idx[idx:idx+nx] = grid_idx[idx:idx+nx][::-1]
      self.grid_idx = grid_idx
      self.calib_grid = dot(grid,self.Tp2w.T)
      self.calib_idx = 0
      
      
      if(os.path.exists("calib_array.npy")):
          self.calib_ang = load("calib_array.npy")
          print(type(self.calib_ang))
          self.move.calibrated = True
          print(self.calib_ang[:,:-1])
        #manual calibration matrix
#      self.calib_ang = [[-1.54566359  0.00872665 -0.03543018]
#                         [ 0.74228853  0.511556   -2.08479579]
#                         [ 2.45148947 -0.61191244  2.08584299]
#                         [-0.80826198  0.66479591 -2.20469991]]
      
      else:
          self.calib_ang = zeros((nx*ny,len(self.arm)))      #This is the matrix you will want to save

    def onEvent(self,evt):
      ###
      ### TEAM CODE GOES HERE
      ###    Handle events as you see fit, and return after

      if evt.type == KEYDOWN:
          progress('in keydown')
          p = "wert".find(evt.unicode)
          if p>=0:
              if self.move.isRunning():
                  progress('Move running!')
                  return
              self.move.pos = self.square_w[p]
              self.move.start()
              self.move.square = True
              self.move.currentPos = self.move.pos
              progress('Move plan started!')
              return
          if evt.key == K_k:
              self.move.prev_pos = self.move.pos
              self.move.pos = self.calib_grid[self.calib_idx]
              self.move.start()
              progress('Moving to calibration point')
              for i,motor in enumerate(self.arm):
                  self.calib_ang[self.calib_idx,i] = motor.get_goal()*(pi/18000)
        #Manual adjustment before this step  
          if evt.key == K_o:
              #Calculate error
              progress('Calculate error')
              if self.calib_idx < len(self.calib_ang):
                  real_ang = zeros(len(self.arm))
                  for i,motor in enumerate(self.arm):
                      real_ang[i] = motor.get_goal()*(pi/18000)   
                  self.calib_ang[self.calib_idx] = real_ang - self.calib_ang[self.calib_idx]
                  self.calib_idx += 1
                  self.move.calDone = True
                  self.move.currentPos = self.move.pos
              progress('Error calculation complete')
              if self.calib_idx == len(self.calib_ang):
                  progress('Calibration_complete!')
                  self.move.calibrated = True
                  save("calib_array.npy",self.calib_ang)
                  return
          #manual movements
          p = "asdf".find(evt.unicode)
          if p>=0:
            progress('manual move')
            self.arm[p].set_pos(self.arm[p].get_goal() + 300)
            return
            # row of 'z' in QWERTY keyboard decrements motors
          p = "zxcv".find(evt.unicode)
          if p>=0:
            progress('manual move')
            self.arm[p].set_pos(self.arm[p].get_goal() - 300)
            return
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
