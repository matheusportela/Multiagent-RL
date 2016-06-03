#  -*- coding: utf-8 -*-
##    @package simulator.py
#      @author Matheus Portela & Guilherme N. Ramos (gnramos@unb.br)
#
# Adapts communication between controller and the Berkeley Pac-man simulator.

from argparse import ArgumentParser
import pickle
import random
import os

from berkeley.graphicsDisplay import PacmanGraphics as BerkeleyGraphics
from berkeley.layout import getLayout as get_berkeley_layout
from berkeley.pacman import runGames as run_berkeley_games
from berkeley.textDisplay import NullGraphics as BerkeleyNullGraphics

import agents
from communication import DEFAULT_CLIENT_ADDRESS, DEFAULT_TCP_PORT
import messages


# Default settings (CLI parsing)
DEFAULT_GHOST_AGENT = 'ai'
DEFAULT_LAYOUT = 'classic'
DEFAULT_NUMBER_OF_GHOSTS = 1
DEFAULT_NUMBER_OF_LEARNING_RUNS = 100
DEFAULT_NUMBER_OF_TEST_RUNS = 15
DEFAULT_OUTPUT_FILE = 'results.txt'
DEFAULT_PACMAN_AGENT = 'random'


def log(msg):
    print '[  Adapter ] {}'.format(msg)


## @todo define pacman-agent choices ad ghost-agent choices from agents.py
# file
#
# @todo define layout choices from layouts dir
def __build_parser__():
    parser = ArgumentParser(description='Run Pac-Man adapter system.')
    parser.add_argument('-g', '--graphics', dest='graphics', default=False,
                        action='store_true',
                        help='display graphical user interface')
    parser.add_argument('-o', '--output', dest='output_filename', type=str,
                        default=DEFAULT_OUTPUT_FILE,
                        help='results output file')

    group = parser.add_argument_group('Experimental Setup')
    group.add_argument('--ghost-agent', dest='ghost_agent', type=str,
                        choices=['random', 'ai'],
                        default=DEFAULT_GHOST_AGENT,
                        help='select ghost agent')
    group.add_argument('-l', '--learn-num', dest='learn', type=int,
                        default=DEFAULT_NUMBER_OF_LEARNING_RUNS,
                        help='number of games to learn from')
    group.add_argument('--layout', dest='layout', type=str,
                        default=DEFAULT_LAYOUT,
                        choices=['classic', 'medium'],
                        help='Game layout')
    group.add_argument('--noise', dest='noise', type=int,
                        default=agents.DEFAULT_NOISE,
                        help='introduce noise in position measurements')
    group.add_argument('--num-ghosts', dest='num_ghosts',
                        type=int, choices=xrange(1, 5),
                        default=DEFAULT_NUMBER_OF_GHOSTS,
                        help='number of ghosts in game')
    group.add_argument('--pacman-agent', dest='pacman_agent',
                        type=str, choices=['random', 'ai', 'eater'],
                        default=DEFAULT_PACMAN_AGENT,
                        help='select Pac-Man agent')
    group.add_argument('--policy-file', dest='policy_filename',
                        type=lambda s: unicode(s, 'utf8'),
                        help='load and save Pac-Man policy from the given'
                        'file')
    group.add_argument('-t', '--test-num', dest='test', type=int,
                        default=DEFAULT_NUMBER_OF_TEST_RUNS,
                        help='number of games to test learned policy')

    group = parser.add_argument_group('Communication')
    group.add_argument('--addr', dest='address', type=str,
                        default=DEFAULT_CLIENT_ADDRESS,
                        help='Client address to connect to adapter')
    group.add_argument('--port', dest='port', type=int,
                        default=DEFAULT_TCP_PORT,
                        help='TCP port to connect to controller')

    return parser

def __get_display__(display_graphics, zoom=1.0, frameTime=0.1):
    if display_graphics:
        return BerkeleyPacmanGraphics(zoom, frameTime=frameTime)
    else:
        return BerkeleyNullGraphics()

def __get_ghost_class__(ghost_agent):
    if ghost_agent == 'random':
        return agents.RandomGhostAgent
    else:  # args.ghost_agent == 'ai':
        return agents.BehaviorLearningGhostAgent

def __get_layout__(layout, num_ghosts):
    LAYOUT_PATH = 'layouts'
    file_name = str(num_ghosts) + 'Ghosts'
    layout_file = '/'.join([LAYOUT_PATH, layout, file_name])

    layout = get_berkeley_layout(layout_file)

    if not layout:
        raise ValueError("The layout " + layout_file + " cannot be found")

    log('Loaded {}.'.format(layout_file))

    return layout

def __get_pacman_class__(pacman_agent):
    if pacman_agent == 'random':
        return agents.RandomPacmanAgent
    elif args.pacman_agent == 'ai':
        return agents.BehaviorLearningPacmanAgent
    else:  # args.pacman_agent == 'eater':
        return agents.EaterPacmanAgent

def __load_policies_from_file__(policy_filename):
    policies = {}
    if policy_filename and os.path.isfile(policy_filename):
        log('Loading policies from {}.'.format(policy_filename))
        with open(policy_filename) as f:
            policies = pickle.loads(f.read())
    return policies

def __write_to_file__(filename, content):
    with open(filename, 'w') as f:
        f.write(pickle.dumps(content))

def create_pacman(agent_class, port):
    pacman = agents.CommunicatingPacmanAgent(port=port)
    pacman.register_agent('pacman', agent_class)
    log('Created {} #{}.'.format(agent_class.__name__, pacman.agent_id))
    return pacman


def create_ghosts(num_ghosts, agent_class, port):
    ghosts = []

    for i in range(num_ghosts):
        ghost = agents.CommunicatingGhostAgent(i+1, port=port)
        ghost.register_agent('ghost', agent_class)
        log('Created {} #{}.'.format(agent_class.__name__, ghost.agent_id))
        ghosts.append(ghost)

    return ghosts


def main():
    parser = __build_parser__()
    args = parser.parse_args()

    agents.NOISE = args.noise

    learn_games = args.learn
    test_games = args.test
    results_output_filename = args.output_filename
    record = False

    pacman_class = __get_pacman_class__(args.pacman_agent)
    ghost_class = __get_ghost_class__(args.ghost_agent)

    layout = __get_layout__(args.layout, args.num_ghosts)
    map_width = layout.width
    map_height = layout.height

    display = __get_display__(args.graphics)

    results = {'learn_scores': [],
               'test_scores': [],
               'behavior_count': {}}

    pacman = create_pacman(pacman_class, args.port)
    ghosts = create_ghosts(args.num_ghosts, ghost_class, args.port)

    ## @todo this as one list, probably by checking if agent is instance of
    # BehaviorLearningAgent (needs refactoring).
    if pacman_class == agents.BehaviorLearningPacmanAgent:
        results['behavior_count'][pacman.agent_id] = {}

    if ghost_class == agents.BehaviorLearningGhostAgent:
        for ghost in ghosts:
            results['behavior_count'][ghost.agent_id] = {}

    # Load policies from file
    policies = __load_policies_from_file__(args.policy_filename)

    # Initialize agents
    pacman.init_agent()
    for ghost in ghosts:
        ghost.init_agent()

    for i in xrange(learn_games + test_games):
        if i >= learn_games:
            log('TEST Game {} (of {})'.format(i+1-learn_games, test_games))
        else:
            log('LEARN Game {} (of {})'.format(i+1, learn_games))

        # Start new game
        pacman.start_game(map_width, map_height)
        for ghost in ghosts:
            ghost.start_game(map_width, map_height)

        # Load policies to agents
        if args.policy_filename and os.path.isfile(args.policy_filename):
            if pacman.agent_id in policies:
                log('Loading {} #{} policy.'.format(type(pacman).__name__,
                                            pacman.agent_id))
                pacman.send_message(messages.PolicyMessage(
                    agent_id=pacman.agent_id,
                    policy=policies[pacman.agent_id]))
                pacman.receive_message()


            for ghost in ghosts:
                if ghost.agent_id in policies:
                    log('Loading {} #{} policy.'.format(type(ghost).__name__,
                                            ghost.agent_id))
                    ghost.send_message(messages.PolicyMessage(
                        agent_id=ghost.agent_id,
                        policy=policies[ghost.agent_id]))
                    ghost.receive_message()

        if i >= learn_games:
            pacman.enable_test_mode()

            for ghost in ghosts:
                ghost.enable_test_mode()

        games = run_berkeley_games(layout, pacman, ghosts, display, 1, record)

        # Do this so as agents can receive the last reward
        pacman.send_message(pacman.create_state_message(games[0].state))
        pacman.receive_message()

        for ghost in ghosts:
            ghost.send_message(ghost.create_state_message(games[0].state))
            ghost.receive_message()

        # Log behavior count
        if pacman_class == agents.BehaviorLearningPacmanAgent:
            msg = messages.RequestBehaviorCountMessage(agent_id=pacman.agent_id)
            pacman.send_message(msg)
            behavior_count_msg = pacman.receive_message()
            log('{} behavior count: {}.'.format(type(pacman).__name__,
                                            behavior_count_msg.count))
            for behavior, count in behavior_count_msg.count.items():
                if behavior not in results['behavior_count'][pacman.agent_id]:
                    results['behavior_count'][pacman.agent_id][behavior] = []
                results['behavior_count'][pacman.agent_id][behavior].append(count)

        if ghost_class == agents.BehaviorLearningGhostAgent:
            for ghost in ghosts:
                msg = messages.RequestBehaviorCountMessage(agent_id=ghost.agent_id)
                ghost.send_message(msg)
                behavior_count_msg = ghost.receive_message()
                log('{} behavior count: {}.'.format(type(ghost).__name__,
                                            behavior_count_msg.count))
                for behavior, count in behavior_count_msg.count.items():
                    if behavior not in results['behavior_count'][ghost.agent_id]:
                        results['behavior_count'][ghost.agent_id][behavior] = []
                    results['behavior_count'][ghost.agent_id][behavior].append(count)

        # Log score
        if i >= learn_games:
            results['test_scores'].append(games[0].state.getScore())
        else:
            results['learn_scores'].append(games[0].state.getScore())

    # Save policies
    if args.policy_filename:
        if pacman_class == agents.BehaviorLearningPacmanAgent:
            pacman.send_message(messages.RequestPolicyMessage(pacman.agent_id))
            msg = pacman.receive_message()
            policies[pacman.agent_id] = msg.policy

        if ghost_class == agents.BehaviorLearningGhostAgent:
            for ghost in ghosts:
                ghost.send_message(messages.RequestPolicyMessage(ghost.agent_id))
                msg = ghost.receive_message()
                policies[ghost.agent_id] = msg.policy

        __write_to_file__(args.policy_filename, policies)

    log('Learn scores: {}'.format(results['learn_scores']))
    log('Test scores: {}'.format(results['test_scores']))

    __write_to_file__(results_output_filename, results)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print '\n\nInterrupted execution\n'
