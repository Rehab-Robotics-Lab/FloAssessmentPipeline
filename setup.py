from distutils.core import setup
from os.path import isdir
from itertools import product

all_packages = ['kinematics']
packages = list(filter(isdir, all_packages))

setup(
    name='kinematics',
    packages=packages,
    version='0.1',
    install_requires=[
            'numpy',
            'scipy'])
