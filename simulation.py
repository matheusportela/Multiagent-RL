#  -*- coding: utf-8 -*-

"""Runs the simulation in different threads for speed.

It assumes the experiment has a package located at "experiments/" containing
a module called "adapter.py". For example, an experiment called "pacman" must
have at least the structure:

experiments/
  pacman/
      __init__.py
      adapter.py
"""

import argparse
from importlib import import_module
import logging
import os
import sys
import threading

from multiagentrl import controller as controller_module

# Logging configuration
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('Simulation')


if __name__ == '__main__':
    # Parse module name
    parser = argparse.ArgumentParser(
        description='Run multiagent reinforcement learning simulation.',
        add_help=False)
    parser.add_argument(
        'module', help='module to execute the simulation (e.g. "pacman")')

    if len(sys.argv) < 2 or sys.argv[1] in ['-h', '--help']:
        parser.print_help()
        exit(1)

    args, unknown = parser.parse_known_args()

    # Parse module-specific arguments
    adapter_module = import_module('experiments.' + args.module + '.adapter')
    adapter_parser = adapter_module.build_parser()
    adapter_args = adapter_parser.parse_args(unknown)

    # Starting simulation
    logger.info('Starting "{}" simulation'.format(args.module))

    # Starting controller
    agents_path = 'experiments.' + args.module + '.agents'
    controller = controller_module.build_controller(agents_path=agents_path)
    controller_thread = threading.Thread(target=controller.run)
    controller_thread.daemon = True
    controller_thread.start()

    # Starting adapter
    adapter = adapter_module.build_adapter_with_args(adapter_args)

    try:
        adapter.run()
    except KeyboardInterrupt:
        logger.info('Interrupted')
        adapter.stop()
