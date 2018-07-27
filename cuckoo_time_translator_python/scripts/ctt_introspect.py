#!/usr/bin/env python
import os
import sys
import numpy as np

from cuckoo_time_translator import *
from cuckoo_time_translator.device_time_bags import *
from cuckoo_time_translator.timestamp_owts import *
from cuckoo_time_translator.batch_algo import printDelayStat

from cuckoo_time_translator.tools import *

OwtsDefault = 'KalmanOwt(), ConvexHullOwt(switchTime = 100)'

if __name__ == '__main__':
  import argparse

  parser = argparse.ArgumentParser(description='Analyze DeviceTimestamp message streams.')
  parser.add_argument('bag', nargs=1, help='The path to the bag file containing the DeviceTimestamp messages')
  parser.add_argument('-t,--topic', dest='topic', nargs='+', help='The path to the bag file containing the DeviceTimestamp messages')
  parser.add_argument('-v,--verbose', dest='verbose', action='count', help='Increase verbosity (counted)')
  parser.add_argument('-o,--output', dest='output', help='Output file to plot to (PDF)')
  parser.add_argument('-b,--baseLine', dest='baseLine', default="LeastSquares", help='Use this batch-method as base line; LeastSquare, ConvexHull, Index')
  parser.add_argument('-f,--owts', dest='owts', default=OwtsDefault, help='Additional owts to compare with. Default: ' + OwtsDefault)
  parser.add_argument('--dontPlotReceiveTimes', action='store_true', help='don\'t plot receive timestamps')
  parser.add_argument('--dontPlotPreTranslated', action='store_true', help='don\'t plot pre-translated timestamps, i.e., translated in the device driver')
  parser.add_argument('--invalidate', action='store_true', help='invalidate any possibly existing cache')
  parser.add_argument('--showDefaults', action='store_true', help='Show all parameters in the legend even if they are at their default value')
  parser.add_argument('--force', action='store_true', help='Force overwriting')

  args = parser.parse_args()
  verbosity = args.verbose
  bagFile = args.bag[0]

  if args.output and os.path.exists(args.output) and not args.force:
    error("Output file " + args.output + " exists already!")
    sys.exit(-1)

  realPathBagFile = os.path.realpath(bagFile)

  if args.topic:
    topics = args.topic
  else:
    topics = guessTopics(bagFile)

  if not topics:
    error("Could not find any topic ending with " + DefaultTopic + "!")
    sys.exit(-1)

  verbose("Going to analyze these topics: " + str(topics))

  hwOwts = eval("[%s]" % args.owts)

  for topic in topics:
    info("Analyzing topic :" + topic)

    ds = DeviceTimeStream(realPathBagFile, topic, invalidate=args.invalidate)

    baselineOwt = None
    if args.baseLine == "Index":
      base_times = np.linspace(ds.receive_times[0], ds.receive_times[-1], len(ds.receive_times))
    elif args.baseLine == "LeastSquares":
      baselineOwt = LeastSquaresOwt()
    elif args.baseLine == "ConvexHull":
      baselineOwt = ConvexHullOwt(True)
    else:
      error("Unknown base line method : " + str(args.baseLine))
      sys.exit(1)

    if baselineOwt:
      base_times = np.array(baselineOwt.apply(ds.raw_hw_times, ds.receive_times))
      info("Baseline owt after translation: " + baselineOwt.getConfigAndStateString())

    delaysToPlot = []
    labels = []
    colors = []

    def addToPlot(times, label, color):
      delaysToPlot.append(times - base_times)
      labels.append(label)
      colors.append(color)

    if not args.dontPlotReceiveTimes:
      addToPlot(ds.receive_times, "receive times", "r")
    if not args.dontPlotPreTranslated:
      addToPlot(ds.translated_hw_times, "pre-translated", "g")

    owtColors = ['m', 'grey', 'cyan', 'k', 'orange']
    for i, owt in enumerate(hwOwts):
        addToPlot(owt.apply(ds.raw_hw_times, ds.receive_times), owt.getConfigString(args.showDefaults), owtColors[i])
        info("After translating: " + owt.getConfigAndStateString())

    print("Deviation from base line:")
    for d, lab in zip(delaysToPlot, labels):
      printDelayStat(d, lab)

    from cuckoo_time_translator.plotting import plotMultiDelays, show
    plotMultiDelays(base_times, delaysToPlot, "time [sec]", labels, markersize=4, colors=colors, fileName=args.output, overwrite=args.force, show=False)

  if not args.output:
    from cuckoo_time_translator.plotting import show
    show()
