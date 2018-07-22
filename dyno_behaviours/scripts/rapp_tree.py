#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE
#
##############################################################################
# Documentation
##############################################################################
"""
Generic launcher for one of the rapp tree entry points. Saves us having to
write and install multiple no-brainer scripts.
"""
##############################################################################
# Imports
##############################################################################

import argparse
import importlib
import py_trees.console as console
import rospy
import sys

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Start one of the trees')
    parser.add_argument('name', action='store', nargs='?',
                        default='route_scheduler',
                        choices=['route_scheduler'],
                        help='name of the rapp tree to start')
    command_line_args = rospy.myargv(argv=sys.argv)[1:]
    args = parser.parse_args(command_line_args)

    module_name = "dyno_behaviours.rapp_trees." + args.name

    try:
        module_itself = importlib.import_module(module_name)
    except ImportError:
        console.logerror("Could not import module [{0}]".format(args.name))
        sys.exit(1)
    main_itself = getattr(module_itself, "main")
    main_itself()
