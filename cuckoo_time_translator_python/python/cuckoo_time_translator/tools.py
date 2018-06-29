from __future__ import print_function

import exceptions
try:
    from termcolor import colored
except exceptions.ImportError:
    print("Unable to import termcolor.")
    print("Try:")
    print("sudo pip install termcolor")

    def colored(X, Y):
        return X

verbosity = False


def info(text):
    print(colored(text, 'yellow'))


def verbose(text):
    if verbosity:
        print(colored(text, 'yellow'))


def warn(text):
    print(colored(str(text), 'red'))


def error(text):
    print(colored("Error :" + str(text), 'red'))


def ok(text):
    print(colored(str(text), 'green'))
