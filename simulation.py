#  -*- coding: utf-8 -*-
#  @file: simulation.py
#  @author: Guilherme N. Ramos (gnramos@unb.br)
#
# Runs the simulation in different threads for speed.
#
# It assumes the experiment has a package located at "experiments/" containing
# a module called "adapter.py". For example, an experiment called "pacman" must
# have at least the structure:
#
# experiments/
#   pacman/
#       __init__.py
#       adapter.py


import argparse
from importlib import import_module
import logging
import os
import threading  # @todo Use multiprocessing instead?

from multiagentrl import controller as controller_module


# Logging configuration
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('Simulation')


MODULE = 'pacman'


if __name__ == '__main__':
    logger.info('Starting "{}" simulation'.format(MODULE))

    adapter_module = import_module('experiments.' + MODULE + '.adapter')
    parser = adapter_module.build_parser()
    args = parser.parse_args()

    controller = controller_module.build_controller()
    controller_thread = threading.Thread(target=controller.run)
    controller_thread.daemon = True
    controller_thread.start()

    adapter = adapter_module.build_adapter_with_args(args)
    adapter_thread = threading.Thread(target=adapter.run)
    adapter_thread.daemon = True
    adapter_thread.start()

    try:
        # Wait adapter process terminate
        while adapter_thread.isAlive():
            # Non-blocking join allows to catch keyboard interrupt
            adapter_thread.join(1)
    except KeyboardInterrupt:
        logger.info('Interrupted')
