#  -*- coding: utf-8 -*-
#       @file: simulation.py
#     @author: Guilherme N. Ramos (gnramos@unb.br)
#
# Runs the simulation.
#
# Assumes a problem module exists in a subdirectory (along with  all its
# associated files), and that it has a cliparser.py file which will provide an
# instance of a Controller and an Adapter.
#
# The simulation is run in different threads for speed.


import argparse
from importlib import import_module
import logging
import os
import threading  # @todo Use multiprocessing instead?

from experiments.pacman import adapter
from multiagentrl import controller


# Logging configuration
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('Simulation')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run Multiagent-RL.', add_help=False,
        usage=argparse.SUPPRESS)
    parser.add_argument('-p', '--path', type=str, default='experiments',
                        help='Path containing experiment packages')
    parser.add_argument('-m', '--module', type=str, default='pacman',
                        choices=['pacman'],
                        help='Name of the package to run the simulation')

    args, unknown = parser.parse_known_args()

    logger.info('Starting "{}" simulation'.format(args.module))

    # @todo spawn one agent per thread

    controller = controller.build_controller()
    controller_thread = threading.Thread(target=controller.run)
    controller_thread.daemon = True
    controller_thread.start()

    adapter = adapter.build_adapter()
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
