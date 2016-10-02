#!/usr/bin/env python
#  -*- coding: utf-8 -*-
"""
Adapts communication between controller and the Berkeley Pac-man simulator.
"""


import pickle
import os
from zmq import Context as zmq_context

from berkeley.graphicsDisplay import PacmanGraphics as BerkeleyGraphics
from berkeley.layout import getLayout as get_berkeley_layout
from berkeley.pacman import runGames as run_berkeley_games
from berkeley.textDisplay import NullGraphics as BerkeleyNullGraphics

import agents
import cliparser

__author__ = "Matheus Portela and Guilherme N. Ramos"
__credits__ = ["Matheus Portela", "Guilherme N. Ramos", "Renato Nobre",
               "Pedro Saman"]
__maintainer__ = "Guilherme N. Ramos"
__email__ = "gnramos@unb.br"


# Default settings (CLI parsing)
DEFAULT_GHOST_AGENT = 'ai'
DEFAULT_LAYOUT = 'classic'
DEFAULT_NUMBER_OF_GHOSTS = 3
DEFAULT_NUMBER_OF_LEARNING_RUNS = 100
DEFAULT_NUMBER_OF_TEST_RUNS = 15
DEFAULT_PACMAN_AGENT = 'random'

# Pac-Man game configuration
NUMBER_OF_BERKELEY_GAMES = 1
RECORD_BERKELEY_GAMES = False


def log(msg):
    print '[  Adapter ] {}'.format(msg)


# @todo Parse arguments outside class, pass values as arguments for
# constructor.
class Adapter(object):
    # @todo define pacman-agent choices and ghost-agent choices from agents.py
    # file
    def __init__(self,
                 pacman_agent=DEFAULT_PACMAN_AGENT,
                 ghost_agent=DEFAULT_GHOST_AGENT,
                 num_ghosts=DEFAULT_NUMBER_OF_GHOSTS,
                 noise=agents.DEFAULT_NOISE,
                 policy_file=None,
                 layout_map=DEFAULT_LAYOUT,
                 learn_runs=DEFAULT_NUMBER_OF_LEARNING_RUNS,
                 test_runs=DEFAULT_NUMBER_OF_TEST_RUNS,
                 output_file=None,
                 graphics=False,
                 context=None, connection=None):

        # Layout ##############################################################
        LAYOUT_PATH = 'pacman/layouts'
        file_name = str(num_ghosts) + 'Ghosts'
        layout_file = '/'.join([LAYOUT_PATH, layout_map + file_name])
        self.layout = get_berkeley_layout(layout_file)
        if not self.layout:
            raise ValueError('Layout {} missing.'.format(layout_file))
        log('Loaded {}.'.format(layout_file))

        # Pac-Man #############################################################
        if pacman_agent == 'random':
            self.pacman_class = agents.RandomPacmanAgent
        elif pacman_agent == 'random2':
            self.pacman_class = agents.RandomPacmanAgentTwo
        elif pacman_agent == 'ai':
            self.pacman_class = agents.BehaviorLearningPacmanAgent
        elif pacman_agent == 'eater':
            self.pacman_class = agents.EaterPacmanAgent
        else:
            raise ValueError
            ('Pac-Man agent must be ai, random, random2 or eater.')

        if not (context and connection):
            context = zmq_context()
            connection = 'tcp://{}:{}'.format(address, port)

        self.pacman = agents.PacmanAdapterAgent(context, connection)
        self.pacman.register('pacman', self.pacman_class)

        # Ghosts ##############################################################
        self.num_ghosts = int(num_ghosts)
        if not (1 <= self.num_ghosts <= 4):
            raise ValueError('Must 1-4 ghost(s).')

        if ghost_agent == 'random':
            self.ghost_class = agents.RandomGhost
        elif ghost_agent == 'ai':
            self.ghost_class = agents.BehaviorLearningGhostAgent
        else:
            raise ValueError('Ghost agent must be ai or random.')

        ghost_name = self.ghost_class.__name__
        self.ghosts = []
        for x in xrange(num_ghosts):
            ghost = agents.GhostAdapterAgent(x + 1, client=client)
            log('Created {} #{}.'.format(ghost_name, ghost.agent_id))
            self.__register_agent__(ghost, 'ghost', self.ghost_class)
            self.ghosts.append(ghost)

        self.all_agents = [self.pacman] + self.ghosts

        # Policies ############################################################
        self.policy_file = str(policy_file) if policy_file else None

        # Runs ################################################################
        self.learn_runs = int(learn_runs)
        assert self.learn_runs > 0

        self.test_runs = int(test_runs)
        assert self.test_runs > 0

        # Output ##############################################################
        if output_file:
            self.output_file = str(output_file)
        else:
            self.output_file = '{}_{}_{}_{}.res'.format(pacman_agent,
                                                        layout_map,
                                                        num_ghosts,
                                                        ghost_agent)

        # Graphical interface #################################################
        if graphics:
            self.display = BerkeleyGraphics()
        else:
            self.display = BerkeleyNullGraphics()

        log('Ready!')

    def __load_policies_from_file__(self, filename):
        policies = {}
        if filename and os.path.isfile(filename):
            log('Loading policies from {}.'.format(filename))
            with open(filename) as f:
                policies = pickle.loads(f.read())
        return policies

    def __log_behavior_count__(self, agent, results):
        behavior_count = agent.get_behavior_count()

        for behavior, count in behavior_count.items():
            if behavior not in results['behavior_count'][agent.agent_id]:
                results['behavior_count'][agent.agent_id][behavior] = []
            results['behavior_count'][agent.agent_id][behavior].append(count)

    def __process_game__(self, policies, results):
        # Start new game
        for agent in self.all_agents:
            agent.start_game(self.layout)

        # Load policies to agents
        if self.policy_file:
            for agent in self.all_agents:
                if agent.agent_id in policies:
                    agent.load_policy(policies[agent.agent_id])

        log('Simulating game...')
        simulated_game = run_berkeley_games(self.layout, self.pacman,
                                            self.ghosts, self.display,
                                            NUMBER_OF_BERKELEY_GAMES,
                                            RECORD_BERKELEY_GAMES)[0]

        # Do this so as agents can receive the last reward
        for agent in self.all_agents:
            agent.update(simulated_game.state)

        # @todo this as one list, probably by checking if agent is
        # instance of BehaviorLearningAgent (needs refactoring).

        # Log behavior count
        if self.pacman_class == agents.BehaviorLearningPacmanAgent:
            self.__log_behavior_count__(self.pacman, results)

        if self.ghost_class == agents.BehaviorLearningGhostAgent:
            for ghost in self.ghosts:
                self.__log_behavior_count__(ghost, results)

        # Log score
        return simulated_game.state.getScore()

    def __save_policies__(self, policies):
        if self.pacman_class == agents.BehaviorLearningPacmanAgent:
            # @todo keep policy in agent?
            policies[self.pacman.agent_id] = self.__get_policy__(self.pacman)

        if self.ghost_class == agents.BehaviorLearningGhostAgent:
            for ghost in self.ghosts:
                policies[ghost.agent_id] = ghost.get_policy()

        self.__write_to_file__(self.policy_file, policies)

    def __write_to_file__(self, filename, content):
        log('Saving results to {}'.format(filename))
        with open(filename, 'w') as f:
            f.write(pickle.dumps(content))

    def run(self):
        log('Now running')

        results = {'learn_scores': [], 'test_scores': [], 'behavior_count': {}}

        # @todo this as one list, probably by checking if agent is instance of
        # BehaviorLearningAgent (needs refactoring).
        if self.pacman_class == agents.BehaviorLearningPacmanAgent:
            results['behavior_count'][self.pacman.agent_id] = {}

        if self.ghost_class == agents.BehaviorLearningGhostAgent:
            for ghost in self.ghosts:
                results['behavior_count'][ghost.agent_id] = {}

        # Load policies from file
        policies = self.__load_policies_from_file__(self.policy_file)

        # Initialize agents
        for agent in self.all_agents:
            agent.initialize()
            # self.__initialize__(agent)

        for x in xrange(self.learn_runs):
            log('LEARN game {} (of {})'.format(x + 1, self.learn_runs))

            score = self.__process_game__(policies, results)
            results['learn_scores'].append(score)

        for agent in self.all_agents:
            agent.enable_test_mode()

        for x in xrange(self.test_runs):
            log('TEST game {} (of {})'.format(x + 1, self.test_runs))

            score = self.__process_game__(policies, results)
            results['test_scores'].append(score)

        if self.policy_file:
            self.__save_policies__(policies)

        log('Learn scores: {}'.format(results['learn_scores']))
        log('Test scores: {}'.format(results['test_scores']))

        self.__write_to_file__(self.output_file, results)

if __name__ == '__main__':
    try:
        adapter = cliparser.get_Adapter()
        adapter.run()
    except KeyboardInterrupt:
        print '\n\nInterrupted execution\n'
