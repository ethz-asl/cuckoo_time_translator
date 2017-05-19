from __future__ import print_function

import numpy as np
import sys

def chunks(l, n):
    """Yield successive n-sized chunks from l."""
    for i in range(0, len(l), n):
        yield l[i:i + n]

def printDelayStat(delays, name, outlierLimit = None, file = None):
    if delays is not None and len(delays):
        if outlierLimit:
            delays = delays[delays < outlierLimit]
            
        delays = np.array(delays) * 1e3 # convert to ms
        
        files = [sys.stdout]
        if file: files.append(file)
        for f in files:
            print("mean(%s) = %g ms" % (name, np.mean(delays)), file = f)
            print("std(%s) = %g ms" %(name, np.std(delays)), file = f)
            for n in (2, 5, 10):
                print("std(%s over %d groups) = %g ms" %(name, n, np.std([ np.mean(c) for c in chunks(delays, n)])), file = f)

    else :
        print("No %s data!" % name)
