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


from argparse import ArgumentParser, SUPPRESS
from importlib import import_module
import threading  ## @todo Use multiprocessing instead?
from zmq import Context as zmq_Context

from communication import InprocServer, InprocClient


def get_module_name():
    """Gets the module name for the problem form the CLI arguments."""
    parser = ArgumentParser(description='Run Multiagent-RL.', add_help=False,
                            usage=SUPPRESS)
    parser.add_argument('-m', '--module', type=str, default='pacman',
                        choices=['pacman'],
                        help='name of the module to run the simulation')
    args, unknown = parser.parse_known_args()
    return args.module


if __name__ == '__main__':
    module_name = get_module_name()

    context = zmq_Context()
    ## @todo one client per thread

    parser_module = import_module(module_name + '.cliparser')
    get_Controller = getattr(parser_module, 'get_Controller')
    controller = get_Controller(context, module_name)
    controller_thread = threading.Thread(target=controller.run)
    controller_thread.daemon = True
    controller_thread.start()

    get_Adapter = getattr(parser_module, 'get_Adapter')
    adapter = get_Adapter(context, module_name)
    adapter_thread = threading.Thread(target=adapter.run)
    adapter_thread.start()
    adapter_thread.join()  # block until adapter process terminates
