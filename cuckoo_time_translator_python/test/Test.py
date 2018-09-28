#!/usr/bin/env python
import os
from cuckoo_time_translator_python.algorithms import *

import unittest

from nose import SkipTest


class TestTimes(unittest.TestCase):

  def testConversions(self):
    l = LocalTime(3)
    self.assertEqual(3, float(l))

  def _testOwt(self, c):
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
    self._testOwt(ConvexHullOwt())

  def testKalman(self):
    self._testOwt(KalmanOwt())


if __name__ == '__main__':
    unittest.main()
