import ikpy.chain
import ikpy.utils.plot as plot_utils

import numpy as np 
import time
import math

import ipywidgets as widgets
import serial

my_chain = ikpy.chain.Chain.from_urdf_file("simon-v2-0.urdf",active_links_mask=[False, False, True, True, True, True, True, True, True, True])

def doIK():
    global ik
    old_position= ik.copy()
    ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Z", initial_position=old_position)
    
def move(x,y,z):
    global target_position
    target_position = [x,y,z]
    doIK()
