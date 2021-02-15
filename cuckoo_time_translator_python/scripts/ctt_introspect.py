#!/usr/bin/env python
import os
import sys
import numpy as np

from cuckoo_time_translator_python import *
from cuckoo_time_translator_python.device_time_bags import *
from cuckoo_time_translator_python.timestamp_owts import *
from cuckoo_time_translator_python.batch_algo import printDelayStat

from cuckoo_time_translator_python.tools import *

OwtsDefault = 'KalmanOwt(), ConvexHullOwt(switchTime = 100)'

if __name__ == '__main__':
  import argparse

  parser = argparse.ArgumentParser(description='Analyze DeviceTimestamp message streams.')
  parser.add_argument('bag', nargs=1, help='The path to the bag file containing the DeviceTimestamp messages')
  parser.add_argument('-t', '--topic', nargs='+', help='The path to the bag file containing the DeviceTimestamp messages')
  parser.add_argument('-v', '--verbose', action='count', help='Increase verbosity (counted)')
  parser.add_argument('-o', '--output', help='Output file to plot to (PDF format)')
  parser.add_argument('-b', '--baseLine', default="LeastSquares", help='Use this batch-method as base line; LeastSquares, ConvexHull, Index, ReceiveTime, DeviceTime')
  parser.add_argument('-f', '--owts', default=OwtsDefault, help='Additional OWTs (one way translators) to compare with. Default: ' + OwtsDefault)
  parser.add_argument('-x', '--x-offset', dest='xOffset', nargs='?', help='Set x-axis (base line) offset that is added to all x-values before plotting. Default is -minimum(x-values), which makes it start at 0.')
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
    elif args.baseLine == "ReceiveTime":
      baselineOwt = ReceiveTimePassThroughOwt(False)
    elif args.baseLine == "DeviceTime":
      baselineOwt = DeviceTimePassThroughOwt(False)
    else:
      error("Unknown base line method : " + str(args.baseLine))
      sys.exit(1)

    if baselineOwt:
      base_times = np.array(baselineOwt.apply(ds.raw_hw_times, ds.receive_times))
      info("Baseline OWT after translation: " + baselineOwt.getConfigAndStateString())

    xOffset = args.xOffset and float(args.xOffset)
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
      addToPlot(ds.translated_hw_times, "pre-translated" + (" (+ zero offset)" if ds.zeroOffsetAllTheTime else " + offset"), "g")
      if not ds.zeroOffsetAllTheTime:
          addToPlot(ds.translated_hw_times_without_offset, "pre-translated without offset", "b")

    owtColors = ['m', 'grey', 'cyan', 'k', 'orange']
    for i, owt in enumerate(hwOwts):
        addToPlot(owt.apply(ds.raw_hw_times, ds.receive_times), owt.getConfigString(args.showDefaults), owtColors[i])
        info("After translating: " + owt.getConfigAndStateString())

    print("Deviation from base line:")
    for d, lab in zip(delaysToPlot, labels):
      printDelayStat(d, lab)

    from cuckoo_time_translator_python.plotting import plotMultiDelays, show
    plotMultiDelays(
        base_times, delaysToPlot, "time [sec]", labels, markersize=4,
        colors=colors, fileName=args.output, overwrite=args.force, show=False,
        title=topic, xOffset=xOffset
    )

  if not args.output:
    from cuckoo_time_translator_python.plotting import show
    show()
