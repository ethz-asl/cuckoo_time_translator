from __future__ import print_function

import numpy as np
import sys


def chunks(l, n):
    """Yield successive n-sized chunks from l."""
    for i in range(0, len(l), n):
        yield l[i:i + n]


def printDelayStat(delays, name, outlierLimit=None, file=None, chunkSizes=()):
    if delays is not None and len(delays):
        if outlierLimit:
            delays = delays[delays < outlierLimit]

        delays = np.array(delays) * 1e3  # convert to ms

        files = [sys.stdout]
        if file: files.append(file)
        for f in files:
            print("%s: mean=%f ms, std=%f ms" % (name, np.mean(delays), np.std(delays)), file=f)
            for n in chunkSizes:
                print("std(%s over %d chunks) = %g ms" % (name, n, np.std([ np.mean(c) for c in chunks(delays, n)])), file=f)

    else :
        print("No %s data!" % name)
