from __future__ import print_function

import numpy as np
import math
import sys

import cuckoo_time_translator as ctt
from cuckoo_time_translator import LocalTime, RemoteTime

class TimestampFilter:
  def __init__(self, owt, batch = False, switchTime = None):
    if switchTime:
      self.owt = ctt.SwitchingOwt(switchTime, owt)
    else:
      self.owt = owt
    self.batch = batch
    self.switchTime = switchTime
    self.name = self.__class__.__name__

  def __str__(self):
    return "%s(batch=%d, switchTime=%s)" %  (self.name, self.batch, str(self.switchTime))

  def apply(self, hwTimes, receiveTimes):
    self.owt.reset()
    assert(len(hwTimes) > 2)
    assert(len(hwTimes) == len(receiveTimes))
    correctedhwTimes = []

    timeScale = (receiveTimes[-1] - receiveTimes[0]) / (hwTimes[-1] - hwTimes[0])

    for ht, rt in zip(hwTimes, receiveTimes):
      correctedhwTimes.append(float(self.owt.updateAndTranslateToLocalTimestamp(RemoteTime(ht * timeScale) , LocalTime(rt))))

    if self.batch:
        correctedhwTimes = []
        for ht, rt in zip(hwTimes, receiveTimes):
            correctedhwTimes.append(float(self.owt.translateToLocalTimestamp(RemoteTime(ht * timeScale))))

    return correctedhwTimes

class ConvexHullFilter (TimestampFilter):
    def __init__(self, *args, **kwargs):
      TimestampFilter.__init__(self, ctt.ConvexHullOwt(), *args, **kwargs)

class KalmanFilter(TimestampFilter):
    def __init__(self, *args, **kwargs):
      TimestampFilter.__init__(self, ctt.KalmanOwt(), *args, **kwargs)
