#!/usr/bin/env python
#  -*- coding: utf-8 -*-
"""
Adapts communication between controller and the Berkeley Pac-man simulator.


"""

import pickle
import os

from berkeley.graphicsDisplay import PacmanGraphics as BerkeleyGraphics
from berkeley.layout import getLayout as get_berkeley_layout
from berkeley.pacman import runGames as run_berkeley_games
from berkeley.textDisplay import NullGraphics as BerkeleyNullGraphics

import agents
import communication as comm

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
DEFAULT_OUTPUT_FILE = 'results.txt'
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
                 layout=DEFAULT_LAYOUT,
                 learn_runs=DEFAULT_NUMBER_OF_LEARNING_RUNS,
                 test_runs=DEFAULT_NUMBER_OF_TEST_RUNS,
                 client=None,
                 output_file=DEFAULT_OUTPUT_FILE,
                 graphics=False):
        # Setup layout
        LAYOUT_PATH = 'pacman/layouts'
        file_name = str(num_ghosts) + 'Ghosts'
        layout_file = '/'.join([LAYOUT_PATH, layout, file_name])
        self.layout = get_berkeley_layout(layout_file)
        if not self.layout:
            raise ValueError('Layout {} missing.'.format(layout_file))
        log('Loaded {}.'.format(layout_file))

        # Setup Pac-Man agent
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

        self.pacman = agents.PacmanAdapterAgent(client=client)
        log('Created {} #{}.'.format(self.pacman_class.__name__,
                                     self.pacman.agent_id))
        self.__register_agent__(self.pacman, 'pacman', self.pacman_class)

        # Setup Ghost agents
        self.num_ghosts = int(num_ghosts)
        if not (1 <= self.num_ghosts <= 4):
            raise ValueError('Must 1-4 ghost(s).')

        if ghost_agent == 'random':
            self.ghost_class = agents.RandomGhostAgent
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

        # Setup policy file
        self.policy_file = str(policy_file) if policy_file else None

        # Setup runs
        self.learn_runs = int(learn_runs)
        if self.learn_runs < 1:
            raise ValueError('Number of learning runs must be at least 1.')

        self.test_runs = int(test_runs)
        if self.test_runs < 1:
            raise ValueError('Number of test runs must be at least 1.')

        self.output_file = str(output_file)

        if graphics:
            self.display = BerkeleyGraphics()
        else:
            self.display = BerkeleyNullGraphics()

        log('Ready')

    def __initialize__(self, agent):
        msg = comm.RequestInitializationMessage(agent_id=agent.agent_id)
        agent.communicate(msg)

    def __get_behavior_count__(self, agent):
        msg = comm.RequestBehaviorCountMessage(agent_id=agent.agent_id)
        reply_msg = agent.communicate(msg)
        return reply_msg.count

    def __get_policy__(self, agent):
        msg = comm.RequestPolicyMessage(agent.agent_id)
        reply_msg = agent.communicate(msg)
        return reply_msg.policy

    def __load_policy__(self, agent, policy):
        msg = comm.PolicyMessage(agent_id=agent.agent_id, policy=policy)
        return agent.communicate(msg)

    def __load_policies_from_file__(self, filename):
        policies = {}
        if filename and os.path.isfile(filename):
            log('Loading policies from {}.'.format(filename))
            with open(filename) as f:
                policies = pickle.loads(f.read())
        return policies

    def __log_behavior_count__(self, agent, results):
        behavior_count = self.__get_behavior_count__(agent)

        for behavior, count in behavior_count.items():
            if behavior not in results['behavior_count'][agent.agent_id]:
                results['behavior_count'][agent.agent_id][behavior] = []
            results['behavior_count'][agent.agent_id][behavior].append(count)

        log('{} behavior count: {}.'.format(type(agent).__name__,
                                            behavior_count))

    def __process_game__(self, policies, results):
        # Start new game
        for agent in self.all_agents:
            agent.start_game(self.layout)

        # Load policies to agents
        if self.policy_file:
            for agent in self.all_agents:
                if agent.agent_id in policies:
                    self.__load_policy__(agent, policies[agent.agent_id])

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

    def __register_agent__(self, agent, agent_team, agent_class):
        log('Request register for {} #{}.'.format(agent_class.__name__,
                                                  agent.agent_id))
        msg = comm.RequestRegisterMessage(agent_id=agent.agent_id,
                                          agent_team=agent_team,
                                          agent_class=agent_class)
        return agent.communicate(msg)

    def __save_policies__(self, policies):
        if self.pacman_class == agents.BehaviorLearningPacmanAgent:
            # @todo keep policy in agent?
            policies[self.pacman.agent_id] = self.__get_policy__(self.pacman)

        if self.ghost_class == agents.BehaviorLearningGhostAgent:
            for ghost in self.ghosts:
                policies[ghost.agent_id] = self.__get_policy__(ghost)

        self.__write_to_file__(self.policy_file, policies)

    def __write_to_file__(self, filename, content):
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
            self.__initialize__(agent)

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
