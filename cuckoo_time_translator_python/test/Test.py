#!/usr/bin/env python

from __future__ import print_function

import os

import unittest
from nose import SkipTest
import numpy as np

from cuckoo_time_translator import *
import cuckoo_time_translator.batch_algo as batch_algo

class TestTimes(unittest.TestCase):
  def testConversions(self):
    l = LocalTime(3)
    self.assertEqual(3, float(l))


  def _testFilter(self, c):
    self.assertFalse(c.isReady())
    l = c.updateAndTranslateToLocalTimestamp(RemoteTime(5), LocalTime(4))
    self.assertEqual(LocalTime, type(l))
    self.assertEqual(4, float(l))
    l = c.updateAndTranslateToLocalTimestamp(RemoteTime(6), LocalTime(5))
    self.assertTrue(c.isReady())
    self.assertEqual(5, float(l))
    self.assertEqual(LocalTime(4), c.translateToLocalTimestamp(RemoteTime(5)))
    self.assertEqual(LocalTime(5), c.translateToLocalTimestamp(RemoteTime(6)))

  def testConvexHull(self):
    self._testFilter(ConvexHullOwt())

  def testKalman(self):
    self._testFilter(KalmanOwt())


  def _testAlign(self, fixSlope):
    x = np.linspace(0, 20, 21);
    y = list(x + np.random.randn(x.shape[0]) * 0.001 * np.mean(np.diff(x)))

    indices = [len(x) / 3 - 1, 2 * len(x) / 3,8]
    indices = sorted(indices)
#     print("indices=", indices)
    x = list(x)
    for i in reversed(indices):
      y.pop(i);

    assert len(x) == len(y) + len(indices)

#     print("x=", x)
#     print("y=", y)
    linModelWithGaps = batch_algo.findGapsAndAffineLinearFit(x, y, fixSlope)

    self.assertEqual(indices, linModelWithGaps.gaps)
    self.assertAlmostEqual(1, linModelWithGaps.slope, places = 3)
    self.assertAlmostEqual(0, linModelWithGaps.offset, places = 2)

    print("err=", batch_algo.calcError(x, y, linModelWithGaps))

  def testAlignFixSlope(self):
    self._testAlign(True)
    
  def testAlign(self):
    self._testAlign(False)

if __name__ == '__main__':
    unittest.main()
