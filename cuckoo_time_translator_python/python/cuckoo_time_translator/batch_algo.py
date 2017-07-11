from __future__ import print_function

import numpy as np
import sys

def chunks(l, n):
    """Yield successive n-sized chunks from l."""
    for i in range(0, len(l), n):
        yield l[i:i + n]

def printToLogAndFile(text, logFunction=None, file = None):
  if file:
    print(text, file = file)
  if logFunction:
    logFunction(text)
  else:
    print(text)

def printDelayStat(delays, name, outlierLimit = None, file = None, chunkSizes = (), logFunction = None):
    if delays is not None and len(delays):
        if outlierLimit:
            delays = delays[delays < outlierLimit]
            
        delays = np.array(delays) * 1e3 # convert to ms
        
        printToLogAndFile("%s: mean=%f ms, std=%f ms" % (name, np.nanmean(delays), np.nanstd(delays)), file = file, logFunction = logFunction)
        for n in chunkSizes:
            printToLogAndFile("std(%s over %d chunks) = %g ms" %(name, n, np.std([ np.mean(c) for c in chunks(delays, n)])), file = file, logFunction = logFunction)
    else :
        print("No %s data!" % name)


class LinearModelWithGaps:
  def __init__(self, slope, offset, gaps):
    self.slope = slope
    self.offset = offset
    self.gaps = sorted(gaps)
    self.indexMap = []
    for i, g in enumerate(self.gaps):
      self.indexMap.extend(range(self.gaps[i - 1] - i + 1 if i > 0 else 0, g - i))
      self.indexMap.append(None)

  def mapXIndexToY(self, ix):
    if ix is None or ix < 0 : return None
    
    if ix >= len(self.indexMap):
      return ix - len(self.gaps)
    else:
      return self.indexMap[ix]
    
  def apply(self, i, x):
    yIndex = self.mapXIndexToY(i)
    if i is None:
      return None, None
    else:
      return yIndex, self.slope * x + self.offset
    
  def __str__(self):
    return "LMWG(s=%g, off=%g, #gaps=%d)" % (self.slope, self.offset, len(self.gaps))


class IndexShifter():
  def __init__(self, shiftXIndexToYIndex):
    self.shiftXIndexToYIndex = shiftXIndexToYIndex

  def calcShiftedX(self, x):
    if self.shiftXIndexToYIndex >= 0:
      return list(x)
    else:
      return x[-self.shiftXIndexToYIndex:]

  def shiftIX(self, ix, inverse = False):
    if self.shiftXIndexToYIndex >= 0:
      return ix
    else:
      return ix + (-1 if inverse else 1) * self.shiftXIndexToYIndex

  def calcShiftedY(self, y):
    if self.shiftXIndexToYIndex >= 0:
      return y[self.shiftXIndexToYIndex:]
    elif self.shiftXIndexToYIndex < 0:
      return y[:self.shiftXIndexToYIndex]

  def shiftIY(self, iy, inverse = False):
    if self.shiftXIndexToYIndex >= 0:
      return iy - (-1 if inverse else 1) * self.shiftXIndexToYIndex
    else:
      return iy

class ShiftedModel:
  def __init__(self, indexShifter, model):
    assert isinstance(model, LinearModelWithGaps)
    assert isinstance(indexShifter, IndexShifter)
    self.model = model
    self.indexShifter = indexShifter

  def calcCorrespondingY(self, xIndices, y):
    result = np.zeros(len(xIndices))
    for i, ix in enumerate(xIndices):
      yi = self.model.mapXIndexToY(self.indexShifter.shiftIX(ix))
      if yi is not None:
        yi = self.indexShifter.shiftIY(yi, True)
      result[i] = y[yi] if yi is not None and not yi >= len(y) else np.NaN
    return result

def linregress(x, y, fixSlope):
  if fixSlope:
    return (1, np.mean(y - x))
  else:
    from scipy import stats
    slope, offset, r_value, p_value, std_err = stats.linregress(x, y)
    return (slope, offset)

def findGapsAndAffineLinearFit(x, yWithGaps, fixSlope = False):
  x = list(x)
  y = list(yWithGaps)
  
  assert(len(x) >= len(y))
  
  gap_indices = []
  while True:
    xa = np.asarray(x[:len(y)])
    ya = np.asarray(y)
    slope, offset = linregress(xa, ya, fixSlope)
    print("slope=%f, offset=%f" %(slope, offset))
#     print("pred=", xa * slope + offset)
#     
    delta = ya - (xa * slope + offset)
    delta_d = np.diff(delta)
#     print("d=", delta)
#     print("dd=", delta_d)

    print("dd: min=%g, mean= %g, max=%g" % (np.min(delta_d), np.mean(delta_d), np.max(delta_d)))

    candidates = sorted(list(np.argwhere(delta_d >= np.max(delta_d)).flatten())) # .6 * np.mean(np.diff(y)
    print("candidates=", candidates)
    if candidates:
      for i in candidates:
        i = i + len(gap_indices) + 1
        gap_indices.append(i)
      
      for i in reversed(gap_indices):
        x.pop(i)
        if len(x) < len(y):
          y.pop()
    else: 
      break
  return LinearModelWithGaps(slope, offset, gap_indices)

def calcError(x, y, linearModelWithGaps):
#   assert len(x) == len(y) + len(linearModelWithGaps.gaps)
  err = 0
  c = 0
  for i, xi in enumerate(x):
    iy, yHat = linearModelWithGaps.apply(i, xi)
    if iy is not None and iy < len(y):
      err += (yHat - y[iy])**2
#       err += abs(yHat - y[iy])
      c+=1
  return err / c, c

def findBestShiftedModel(xI, yIWithGaps, fixSlope, ran = range(-2, 3)):
  bestShiftedModel = None
  lowestErr = None
  for i in ran:
    indexShifter = IndexShifter(i)
    x = indexShifter.calcShiftedX(xI)
    y = indexShifter.calcShiftedY(yIWithGaps)
    if len(x) < len(y):
      y = y[:len(x)]
    model = findGapsAndAffineLinearFit(x, y, fixSlope=fixSlope)
    err = calcError(x, y, model)
    print("i=", i, ", err=", err, ", model=", model)
    if not lowestErr or lowestErr > err:
      lowestErr, bestShiftedModel = err, ShiftedModel(indexShifter, model)

  print(lowestErr, bestShiftedModel)
  return bestShiftedModel
