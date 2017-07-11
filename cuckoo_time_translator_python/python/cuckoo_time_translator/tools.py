from __future__ import print_function

import exceptions
import sys

try:
    from termcolor import colored
except exceptions.ImportError:
    print("Unable to import termcolor.")
    print("Try:")
    print("sudo pip install termcolor")
    def colored(X,Y):
        return X

verbosity = False

def printColored(text, color, end = '\n'):
  print (colored(str(text), color), end = end)
  if not end:
    sys.stdout.flush();

def info(text, end='\n'):
  print(text, end)

def verbose(text, end='\n'):
  if verbosity:
    printColored(text, 'cyan', end)

def ok(text, end='\n'):
  printColored(text, 'green', end)

def warn(text, end='\n'):
  printColored("Warning: " + str(text), 'magenta', end)

def error(text, end='\n'):
  printColored("Error: " + str(text), 'red', end)


def loadJson(in_file, ignoreMissing = False):
  from json import load
  if ignoreMissing:
    from os.path import exists
    if not exists(in_file):
      return None

  with open(in_file, 'r') as f:
    data = load(f)
  return data

def writeJson(out_file, data):
  from json import dump
  with open(out_file, 'w') as f:
    dump(data, f, indent=3, sort_keys=True)
