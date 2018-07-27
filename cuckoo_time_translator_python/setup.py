
# # ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['cuckoo_time_translator'],
    package_dir={'':'python'},
    scripts=['scripts/ctt_introspect.py']
  )

setup(**setup_args)
