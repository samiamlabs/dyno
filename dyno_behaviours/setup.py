#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['dyno_behaviours'],
    # scripts=['scripts/demo_tree'],
    # package_dir={'': 'src'}
)

setup(**setup_args)
