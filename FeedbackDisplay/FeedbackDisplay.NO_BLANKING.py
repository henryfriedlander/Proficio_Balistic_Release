
import sys
import os
import math
import winsound

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
from PyDragonfly import Dragonfly_Module, CMessage, copy_to_msg, copy_from_msg, MT_EXIT
from dragonfly_utils import respond_to_ping
import Dragonfly_config as rc

MAX_WIDTH = 1280
OFFSET=20


class Display(ColorLayer):
    is_event_handler = True

    def __init__(self):
        super(Display, self).__init__(180, 180, 180, 255)

        self.mod = Dragonfly_Module(0, 0)
        self.mod.ConnectToMMM("everest:7111")
        self.mod.Subscribe(rc.MT_TASK_STATE_CONFIG)
        self.mod.Subscribe(rc.MT_INPUT_DOF_DATA)
        self.mod.Subscribe(rc.MT_COMBO_WAIT)
        self.mod.Subscribe(rc.MT_TRIAL_CONFIG)
        self.mod.Subscribe(rc.MT_END_TASK_STATE)
        self.mod.Subscribe(rc.MT_PING)

        self.msg = CMessage()
        
        self.timer_sec = 0
        self.timer_min = 0
        
        self.position_bar = Polygon(v=[(20, 455), (20, 565), (1260, 565), (1260, 455)], color=(0.05, 0.05, 0.05, 1), stroke=0)
        self.tgt_window   = Polygon(v=[(0, 0), (0, 0), (0, 0), (0, 0)], color=(0, 0.6, 0.2, 1), stroke=0)
        self.pos_fdbk     = Polygon(v=[(20, 405), (20, 615), (28, 615), (28, 405)], color=(0, 0.2, 1.0, 1), stroke=0)
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
                                      color=(0,0,0,255),
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


        self.reset_score()

        self.schedule_interval(self.update, 0.01)

    def timer_count_down(self, dt):
        self.timer_sec -= 1
        
        if self.timer_sec < 0:
            if self.timer_min > 0:
                self.timer_min -= 1
                self.timer_sec = 59
            else:
                self.unschedule(self.timer_count_down)
                self.combo_wait_txt.text = ''
                return
        
        self.combo_wait_txt.text = 'Relax Time   %d:%02d' % (self.timer_min, self.timer_sec)

        if self.timer_min == 0 and self.timer_sec <= 5 and self.timer_sec > 0:
            winsound.PlaySound(os.path.join(os.environ.get('ROBOT_CONFIG'), 'default', 'gocue.wav'), winsound.SND_FILENAME | winsound.SND_ASYNC)

    def reset_score(self):
        self.score = 0
        self.score_force_level = 0
        self.score_target_dist = 999
        self.score_force_mult = 0
        self.score_target_mult = 0
        self.score_txt.text = "Score: %d" % self.score
        self.reward_txt.text = ""

    def update_reward(self):
        reward = self.score_force_mult * self.score_target_mult
        pts = "points"
        if reward == 1:
            pts = "point"
        self.reward_txt.text = "Current Reward: %d %s" % (reward, pts)

    def increment_score(self):
        self.score += self.score_force_mult * self.score_target_mult
        self.score_txt.text = "Score: %d" % self.score

    def update(self, dt):
        while True:
            rcv = self.mod.ReadMessage(self.msg, 0)
            if rcv == 1:
                hdr = self.msg.GetHeader()
                msg_type = hdr.msg_type

                if msg_type == rc.MT_INPUT_DOF_DATA:
                    mdf = rc.MDF_INPUT_DOF_DATA()
                    copy_from_msg(mdf, self.msg)

                    if mdf.tag == 'carduinoIO':
                        fdbk = 5 - mdf.dof_vals[7]  # invert to match phyiscal setup
                        x_pos = int((fdbk * (MAX_WIDTH - 2*OFFSET))/5.0)
                        
                        self.pos_fdbk_txt.text = "%.2f V" % fdbk
                        
                        self.pos_fdbk.v[0] = (x_pos, 405)
                        self.pos_fdbk.v[1] = (x_pos, 615)
                        self.pos_fdbk.v[2] = (x_pos+8, 615)
                        self.pos_fdbk.v[3] = (x_pos+8, 405)

#                if msg_type == rc.MT_FORCE_SENSOR_DATA:
#                    mdf = rc.MDF_FORCE_SENSOR_DATA()
#                    copy_from_msg(mdf, self.msg)
#
#                    x_fdbk = mdf.data[0]
#                    x_fdbk_width = int((x_fdbk / MAX_FDBK) * MAX_WIDTH)

                elif msg_type == rc.MT_COMBO_WAIT:
                    mdf = rc.MDF_COMBO_WAIT()
                    copy_from_msg(mdf, self.msg)
                
                    print mdf.duration
                
                    duration = mdf.duration / 1000      # convert to seconds
                    self.timer_sec = duration % 60
                    self.timer_min = duration / 60
                    self.schedule_interval(self.timer_count_down, 1)
                
                elif msg_type == rc.MT_TRIAL_CONFIG:
                    self.unschedule(self.timer_count_down)
                    self.combo_wait_txt.text = ''

                elif msg_type == rc.MT_PING:
                    self.reset_score()

                elif msg_type == rc.MT_END_TASK_STATE:
                    mdf = rc.MDF_END_TASK_STATE()
                    copy_from_msg(mdf, self.msg)

                    # look for the "Reward" task state
                    if (mdf.id == 5) & (mdf.outcome == 1):
                        self.increment_score()

                elif msg_type == rc.MT_TASK_STATE_CONFIG:
                    mdf = rc.MDF_TASK_STATE_CONFIG()
                    copy_from_msg(mdf, self.msg)
    
                    if mdf.background_color == 'gray':
                        self.color = (180, 180, 180)
                    elif mdf.background_color == 'red':
                        self.color = (150, 12, 12)
                    elif mdf.background_color == 'green':
                        self.color = (0, 150, 50)

                    if mdf.fdbk_display_color == 'gray':
                        self.tgt_window.color = (0.3, 0.3, 0.3, 1)
                    elif mdf.fdbk_display_color == 'yellow':
                        self.tgt_window.color = (0.5, 0.5, 0.0, 1)
                    elif mdf.fdbk_display_color == 'green':
                        self.tgt_window.color = (0.0, 0.6, 0.2, 1)
                    elif mdf.fdbk_display_color == 'red':
                        self.tgt_window.color = (0.6, 0.05, 0.05, 1)

                    if not(math.isnan(mdf.target[0])) and not(math.isnan(mdf.target[1])):
                        x_tgt_lo = int((mdf.target[0] * (MAX_WIDTH - 2*OFFSET))/5.0) 
                        x_tgt_hi = int((mdf.target[1] * (MAX_WIDTH - 2*OFFSET))/5.0) 

                        self.tgt_window.v[0] = (x_tgt_lo, 430)
                        self.tgt_window.v[1] = (x_tgt_lo, 590)
                        self.tgt_window.v[2] = (x_tgt_hi, 590)
                        self.tgt_window.v[3] = (x_tgt_hi, 430)

                        # update multipliers during "ForceRamp" task state
                        if mdf.id == 2:
                            force_level = mdf.sep_threshold_f[1]
                            if force_level > self.score_force_level:
                                self.score_force_level = force_level
                                self.score_force_mult += 1
                                
                                # new combo, reset target multipliers
                                self.score_target_mult = 0
                                self.score_target_dist = 999
                            
                            target_level = mdf.target[1] - mdf.target[0]
                            if target_level < self.score_target_dist:
                                self.score_target_dist = target_level
                                self.score_target_mult += 1

                            self.update_reward()
                                
                            #print mdf.id
                            #print force_level, self.score_force_level, self.score_force_mult
                            #print target_level, self.score_target_dist, self.score_target_mult
                            #print "\n"


            else:
                break    


    #def on_key_release( self, keys, mod ):


    def draw(self):
        super( Display, self).draw()

        self.position_bar.render()
        self.tgt_window.render()
        self.pos_fdbk.render()
        #self.pos_fdbk_txt.draw()
        self.combo_wait_txt.draw()
        self.score_txt.draw()
        self.reward_txt.draw()


if __name__ == "__main__":
    director.init(width=1280, height=1024, caption="FeedbackDisplay", resizable=True, do_not_scale=True) #, fullscreen=True)

    scene = Scene( Display() )
    
    director.run( scene )

