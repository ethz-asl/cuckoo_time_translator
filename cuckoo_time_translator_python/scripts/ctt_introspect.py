#!/usr/bin/env python
import os
import numpy as np

from cuckoo_time_translator import *
from cuckoo_time_translator.device_time_bags import *
from cuckoo_time_translator.timestamp_filters import *
from cuckoo_time_translator.batch_algo import printDelayStat

from cuckoo_time_translator.tools import *

FiltersDefault = 'ConvexHullFilter(switchTime = 10), KalmanFilter(), ConvexHullFilter(switchTime = 100)'

if __name__ == '__main__':
  import argparse
  
  parser = argparse.ArgumentParser(description='Analyze DeviceTimestamp message streams.')
  parser.add_argument('bag', nargs = 1, help='The path to the bag file containing the DeviceTimestamp messages')
  parser.add_argument('-t,--topic', dest='topic', nargs='+', help='The path to the bag file containing the DeviceTimestamp messages')
  parser.add_argument('-v,--verbose', dest='verbose', action='count', help='Increase verbosity (counted)')
  parser.add_argument('-o,--output', dest='output', help='Output file to plot to (PDF)')
  parser.add_argument('-f,--filters', dest='filters', default=FiltersDefault, help='Additional filters to compare with. Default: ' + FiltersDefault)
  parser.add_argument('--dontPlotReceiveTimes', action='store_true', help='don\'t plot receive timestamps')
  parser.add_argument('--dontPlotPreFiltered', action='store_true', help='don\'t plot pre-filtered timestamps')
  parser.add_argument('--useAffineZoom', action='store_true', help='use an affine linear transformation to allow high resolution on the y-axis')
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
  
  hwFilters = eval("[%s]" % args.filters)
  
  for topic in topics:
    info("Analyzing topic :" + topic)

    ds = DeviceTimeStream(realPathBagFile, topic, invalidate = args.invalidate)
  
    if args.useAffineZoom:
      base_times = np.linspace(ds.receive_times[0], ds.receive_times[-1], len(ds.receive_times))
    else:
      base_times = np.array(ConvexHullFilter(True).apply(ds.raw_hw_times, ds.receive_times))
  
    delaysToPlot = []
    labels = []
    colors = []
    
    def addToPlot(times, label, color):
      delaysToPlot.append(times - base_times)
      labels.append(label)
      colors.append(color)

    if not args.dontPlotReceiveTimes:
      addToPlot(ds.receive_times, "receive times", "r")
    if not args.dontPlotPreFiltered:
      addToPlot(ds.filtered_hw_times, "pre-filtered", "g")
  
    filterColors = ['m', 'grey', 'cyan', 'k', 'orange']
    for i, filter in enumerate(hwFilters):
        addToPlot(filter.apply(ds.raw_hw_times, ds.receive_times), filter.getConfigString(args.showDefaults), filterColors[i])
  
    for d, lab in zip(delaysToPlot, labels):
      printDelayStat(d, lab)
  
    from cuckoo_time_translator.plotting import plotMultiDelays, show
    plotMultiDelays(base_times, delaysToPlot, "time [sec]", labels, markersize = 4, colors = colors, fileName = args.output, overwrite = args.force, show = False)

  if not args.output:
    from cuckoo_time_translator.plotting import show
    show()
