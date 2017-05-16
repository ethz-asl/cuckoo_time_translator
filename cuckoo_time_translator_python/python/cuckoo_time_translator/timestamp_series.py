from __future__ import print_function

import os
import sys

import pickle
import rosbag

class TimestampSeries(list):
  def __init__(self):
    pass
    
  def append(self, t):
    assert float == type(t), "type(t)=" + str(type(t))
    list.append(self, t)

