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
import threading  # @todo Use multiprocessing instead?

import zmq

import controller


def log(msg):
    print '[Simulation] {}'.format(msg)


def get_module_name():
    """Gets the module name for the problem form the CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Run Multiagent-RL.', add_help=False,
        usage=argparse.SUPPRESS)
    parser.add_argument('-m', '--module', type=str, default='pacman',
                        choices=['pacman'],
                        help='name of the module to run the simulation')
    args, unknown = parser.parse_known_args()
    return args.module


if __name__ == '__main__':
    module_name = get_module_name()

    log('Starting "{}" simulation'.format(module_name))

    # @todo spawn one agent per thread

    controller = controller.build_controller()
    controller_thread = threading.Thread(target=controller.run)
    controller_thread.daemon = True
    controller_thread.start()

    adapter_module = import_module(module_name + '.adapter')
    build_adapter = getattr(adapter_module, 'build_adapter')
    adapter = build_adapter()
    adapter_thread = threading.Thread(target=adapter.run)
    adapter_thread.daemon = True
    adapter_thread.start()

    try:
        # Wait adapter process terminate
        while adapter_thread.isAlive():
            # Non-blocking join allows to catch keyboard interrupt
            adapter_thread.join(1)
    except KeyboardInterrupt:
        log('Interrupted')
