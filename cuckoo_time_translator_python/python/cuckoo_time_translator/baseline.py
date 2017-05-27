from __future__ import print_function

import numpy as np
import math
import sys

from timestamp_filters import *
from tools import *

class BaseLine:
    def __init__(self):
        pass

class FilterBaseLine(BaseLine):
  def __init__(self, filter):
    self.filter = filter
    
  def compute(self, raw_hw_times, receive_times):
    base_times = np.array(self.filter.apply(raw_hw_times, receive_times))
    info("Baseline filter after filtering: " + self.filter.getConfigAndStateString())
    return base_times

class FilterBaseLine(BaseLine):
  def __init__(self, filter):
    self.filter = filter
    
  def compute(self, raw_hw_times, receive_times):
    base_times = np.array(self.filter.apply(raw_hw_times, receive_times))
    info("Baseline filter after filtering: " + self.filter.getConfigAndStateString())
    return base_times

class IndexBaseLine(BaseLine):
  def compute(self, raw_hw_times, receive_times):
    return np.linspace(receive_times[0], receive_times[-1], len(receive_times))

def create(configString):
    name = configString
    baselineFilter = None
    if name == "Index":
      return IndexBaseLine()

    elif name == "LeastSquares":
      baselineFilter = LeastSquaresFilter()
    elif name == "ConvexHull":
      baselineFilter = ConvexHullFilter(True)
    else:
      error("Unknown base line method : " + str(name))
      sys.exit(1)

    if baselineFilter:
      return FilterBaseLine(baselineFilter)
