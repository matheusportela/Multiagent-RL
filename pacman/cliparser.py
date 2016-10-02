#!/usr/bin/env python
#  -*- coding: utf-8 -*-

"""Parses CLI arguments to provide Adapter and Controller instances."""


from argparse import ArgumentParser
from zmq import Context as zmq_Context

from adapter import (Adapter, DEFAULT_GHOST_AGENT, DEFAULT_LAYOUT,
                     DEFAULT_NUMBER_OF_GHOSTS, DEFAULT_NUMBER_OF_LEARNING_RUNS,
                     DEFAULT_NUMBER_OF_TEST_RUNS, DEFAULT_PACMAN_AGENT)
from agents import DEFAULT_NOISE
from controller import Controller

# @todo properly include communication module from parent folder
import sys
sys.path.insert(0, '..')
from communication import (InprocServer, TCPServer, DEFAULT_CLIENT_ADDRESS,
                           DEFAULT_TCP_PORT)

__author__ = "Matheus Portela and Guilherme N. Ramos"
__credits__ = ["Matheus Portela", "Guilherme N. Ramos", "Renato Nobre",
               "Pedro Saman"]
__maintainer__ = "Guilherme N. Ramos"
__email__ = "gnramos@unb.br"


def get_Adapter(context=None, endpoint=None):
    """Parses arguments and returns an Adapter."""
    parser = ArgumentParser(description='Run Pac-Man adapter system.')
    parser.add_argument('-g', '--graphics', dest='graphics', default=False,
                        action='store_true',
                        help='display graphical user interface')
    parser.add_argument('-o', '--output', dest='output_file', type=str,
                        help='results output file')

    group = parser.add_argument_group('Experimental Setup')
    group.add_argument('--ghost-agent', dest='ghost_agent', type=str,
                       choices=['random', 'ai'], default=DEFAULT_GHOST_AGENT,
                       help='select ghost agent')
    group.add_argument('-l', '--learn-num', dest='learn_runs', type=int,
                       default=DEFAULT_NUMBER_OF_LEARNING_RUNS,
                       help='number of games to learn from')
    group.add_argument('--layout', dest='layout', type=str,
                       default=DEFAULT_LAYOUT, choices=['classic', 'medium'],
                       help='Game layout')
    group.add_argument('--noise', dest='noise', type=int,
                       default=DEFAULT_NOISE,
                       help='introduce noise in position measurements')
    group.add_argument('--num-ghosts', dest='num_ghosts',
                       type=int, choices=range(1, 5),
                       default=DEFAULT_NUMBER_OF_GHOSTS,
                       help='number of ghosts in game')
    group.add_argument('--pacman-agent', dest='pacman_agent', type=str,
                       choices=['random', 'random2', 'ai', 'eater'],
                       default=DEFAULT_PACMAN_AGENT,
                       help='select Pac-Man agent')
    group.add_argument('--policy-file', dest='policy_file',
                       type=lambda s: unicode(s, 'utf8'),
                       help='load and save Pac-Man policy from the given file')
    group.add_argument('-t', '--test-num', dest='test_runs', type=int,
                       default=DEFAULT_NUMBER_OF_TEST_RUNS,
                       help='number of games to test learned policy')

    group = parser.add_argument_group('Communication')
    group.add_argument('--addr', dest='address', type=str,
                       default=DEFAULT_CLIENT_ADDRESS,
                       help='Client address to connect to adapter (TCP '
                            'connection)')
    group.add_argument('--port', dest='port', type=int,
                       default=DEFAULT_TCP_PORT,
                       help='Port to connect to controller (TCP connection)')

    args, unknown = parser.parse_known_args()

    if context and endpoint:
        connection = 'inproc://{}'.format(endpoint)
    else:
        context = zmq_Context()
        connection = 'tcp://{}:{}'.format(args.address, args.port)

    adapter = Adapter(pacman_agent=args.pacman_agent,
                      ghost_agent=args.ghost_agent,
                      num_ghosts=args.num_ghosts,
                      noise=args.noise,
                      policy_file=args.policy_file,
                      layout_map=args.layout,
                      learn_runs=args.learn_runs,
                      test_runs=args.test_runs,
                      output_file=args.output_file,
                      graphics=args.graphics,
                      context=context, connection=connection)

    return adapter


def get_Controller(context=None, endpoint=None):
    """Parses arguments and returns a Controller.

    If no server is given, instantiates a TCPServer."""
    parser = ArgumentParser(description='Run Pac-Man controller system.')
    parser.add_argument('--port', dest='port', type=int,
                        default=DEFAULT_TCP_PORT,
                        help='TCP port to connect to adapter')
    args, unknown = parser.parse_known_args()

    # @todo setup an option for a "memory" server (direct communication with
    # Adapter) (zmq inproc?)

    if context and endpoint:
        binding = 'inproc://{}'.format(endpoint)
    else:
        context = zmq_Context()
        binding = 'tcp://*:{}'.format(args.port)

    return Controller(context, binding)
