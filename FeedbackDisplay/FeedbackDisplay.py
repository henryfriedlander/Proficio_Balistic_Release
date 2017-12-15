# add a message that stores which direction of the trial (X,Y,Z)
# add info whether it is forward or backward


import sys
import os
import math

from cocos.actions import *
from cocos.director import director
from cocos.layer import Layer, ColorLayer
from cocos.scene import Scene
from cocos.sprite import Sprite
from cocos.text import Label

from primitives import Polygon

import pyglet
from pyglet.gl import *


from ConfigParser import SafeConfigParser
from argparse import ArgumentParser
from PyDragonfly import Dragonfly_Module, CMessage, copy_to_msg, copy_from_msg, read_msg_data, MT_EXIT
from dragonfly_utils import respond_to_ping
import Dragonfly_config as rc

MAX_WIDTH = 1280
OFFSET=20


BEGINTRIAL_TS = 1
FORCERAMP_TS = 2
REWARD_TS = 5
transformType = 1


class Display(ColorLayer):
  is_event_handler = True

  def __init__(self):
      super(Display, self).__init__(180, 180, 180, 255)

      self.XorYorZ = 0
      self.UporDown = 1
      self.tform = 1

      self.mod = Dragonfly_Module(0, 0)
      self.mod.ConnectToMMM()
      self.mod.Subscribe(rc.MT_TASK_STATE_CONFIG)
      self.mod.Subscribe(rc.MT_TRIAL_CONFIG)
      self.mod.Subscribe(rc.MT_TRIAL_INPUT)
      self.mod.Subscribe(rc.MT_RT_POSITION_FEEDBACK)

      self.msg = CMessage()
      
      self.timer_sec = 0
      self.timer_min = 0
      
      dims = director.get_window_size()
      self.width = dims[0]
      self.height = dims[1]
      self.oldWidth = 1280
      self.oldHeight = 1024
      
      self.blank_display = False
      
      self.position_bar = Polygon(v=[(20, 585), (20, 695), (1260, 695), (1260, 585)], 
        color=(0.05, 0.05, 0.05, 1), stroke=0)    
      
      self.tgt_window = Polygon(v=[(0, 0), (0, 10), (10, 10), (10, 0)], 
        color=(0, 0.6, 0.2, 1), stroke=0)
      self.resizePolygon(self.tgt_window)
      
      self.pos_fdbk = Polygon(v=[(20, 535), (20, 745), (28, 745), (28, 535)], 
        color=(0, 0.2, 1.0, 1), stroke=0)
      self.resizePolygon(self.pos_fdbk)
      
      self.schedule_interval(self.update, 0.001)
      
      self.pos_fdbk_txt = pyglet.text.Label('',
                                    font_name='times new roman',
                                    font_size=32,
                                    color=(0,0,0,255),
                                    width=250,
                                    bold=True,
                                    x=1140, y=40)

      self.combo_wait_txt = pyglet.text.Label('',
                                    font_name='times new roman',
                                    font_size=32,
                                    color=(255,255,255,255),
                                    width=250,
                                    bold=True,
                                    x=478, y=840)
                                    
      self.score_txt = pyglet.text.Label('',
                                    font_name='times new roman',
                                    font_size=36,
                                    color=(0,0,0,255),
                                    width=250,
                                    bold=True,
                                    x=533, y=240)                                      

      self.reward_txt = pyglet.text.Label('',
                                    font_name='times new roman',
                                    font_size=22,
                                    color=(0,0,0,255),
                                    width=250,
                                    bold=True,
                                    x=475, y=120)                                      

  def screen_off(self):
    self.color = (0, 0, 0)
    self.blank_display = True
  
  def screen_on(self):
    self.blank_display = False
    self.color = (180, 180, 180)
      
  def update(self, dt):
    while True:
      seen = False
      rcv = self.mod.ReadMessage(self.msg, 0)
      if rcv == 1:
        #print "recieved a message..."
        hdr = self.msg.GetHeader()
        msg_type = hdr.msg_type

        if msg_type == rc.MT_TRIAL_INPUT:
          # Starting a new trial
          mdf = rc.MDF_TRIAL_INPUT()
          copy_from_msg(mdf, self.msg)
          self.XorYorZ = mdf.XorYorZ
          self.UporDown = mdf.UpOrDown
          self.tform = self.UporDown + self.XorYorZ
          print "Starting new trial xyz: ",self.XorYorZ,", UoD: ",self.UporDown
          seen = False
              

        # updates real time position of handle on screen; receives 
        # messages from cube_sphere while loop
        elif msg_type == rc.MT_RT_POSITION_FEEDBACK: 
          mdf = rc.MDF_RT_POSITION_FEEDBACK()
          copy_from_msg(mdf, self.msg)

          x_pos = mdf.distanceFromCenter;
          print "X_pos", x_pos

          #for i in xrange(4):
          #    self.pos_fdbk.v[i] = self.transformPoints(self.pos_fdbk.v[i], 0)
            
          self.pos_fdbk.v[0] = self.transformPoints((x_pos, 535), self.tform)
          self.pos_fdbk.v[1] = self.transformPoints((x_pos, 745), self.tform)
          self.pos_fdbk.v[2] = self.transformPoints((x_pos+8, 745), self.tform)
          self.pos_fdbk.v[3] = self.transformPoints((x_pos+8, 535), self.tform)


        elif msg_type == rc.MT_TASK_STATE_CONFIG:
          print "received data"
          mdf = rc.MDF_TASK_STATE_CONFIG()
          copy_from_msg(mdf, self.msg)
            
          # Update background color
          if mdf.background_color == 'gray':
            self.color = (180, 180, 180)
          elif mdf.background_color == 'red':
            self.color = (150, 12, 12)
          elif mdf.background_color == 'green':
            self.color = (0, 150, 50)

          # Change target color
          if mdf.fdbk_display_color == 'gray':
            self.tgt_window.color = (0.5, 0.5, 0.0, 1) #this is yellow, remove
            #self.tgt_window.color = (0.3, 0.3, 0.3, 1)
          elif mdf.fdbk_display_color == 'yellow':
            self.tgt_window.color = (0.5, 0.5, 0.0, 1) #this is green....
          elif mdf.fdbk_display_color == 'green':
            self.tgt_window.color = (0.0, 0.6, 0.2, 1)
          elif mdf.fdbk_display_color == 'red':
            self.tgt_window.color = (0.6, 0.05, 0.05, 1)
          if not seen:
            x_tgt_lo = mdf.target[0]
            x_tgt_hi = mdf.target[1]
            # Transform target window
            self.tgt_window.v[0] = self.transformPoints((x_tgt_lo, 560), self.tform)
            self.tgt_window.v[1] = self.transformPoints((x_tgt_lo, 720), self.tform)
            self.tgt_window.v[2] = self.transformPoints((x_tgt_hi, 720), self.tform)
            self.tgt_window.v[3] = self.transformPoints((x_tgt_hi, 560), self.tform)
            # Transform position bar
            self.position_bar.v[0] = self.transformPoints((20, 585), self.tform)
            self.position_bar.v[1] = self.transformPoints((20, 695), self.tform)
            self.position_bar.v[2] = self.transformPoints((1260, 695), self.tform)
            self.position_bar.v[3] = self.transformPoints((1260, 585), self.tform)
            seen = True


  def transformPolygon(self, polygon, transformationType):
    for i in xrange(4):
      polygon.v[i] = self.transformPoint(polygon.v[i], transformationType)

  def transformPoints(self, input_point, tt):
    # tt values are determined by adding XorYorZ and UpOrDown
    if tt == 0: # goes from left to right XorYorZ=1 UpOrDown=-1 
      return input_point
    elif tt == 2: # goes from right to left XorYorZ=1 UpOrDown=1
      return (self.width - input_point[0], input_point[1])
    elif tt == 1: # backward XorYorZ=0 UpOrDown=1
      return (input_point[1], self.width - input_point[0])
    elif tt == -1: # forwards XorYorZ=0 UpOrDown=-1
      return (input_point[1], input_point[0])
    else:
      print "unknown transform type"
  
  def resizePolygon(self, polygon):
    for i in xrange(4):
      polygon.v[i] = (polygon.v[i][0] * self.width / self.oldWidth, polygon.v[i][1] * self.height / self.oldHeight)

  def draw(self):
    super( Display, self).draw()

    if not self.blank_display:
      self.position_bar.render()
      self.tgt_window.render()
      self.pos_fdbk.render()
      #self.pos_fdbk_txt.draw()
      self.score_txt.draw()
      self.reward_txt.draw()

    self.combo_wait_txt.draw()


if __name__ == "__main__":
    director.init(width=700, height=800, caption="FeedbackDisplay", resizable=True, autoscale=False) #, fullscreen=True)


    scene = Scene( Display() )
    
    director.run( scene )

