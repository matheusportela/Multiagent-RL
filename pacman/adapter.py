#  -*- coding: utf-8 -*-
##    @package simulator.py
#      @author Matheus Portela & Guilherme N. Ramos (gnramos@unb.br)
#
# Adapts communication between controller and the Berkeley Pac-man simulator.

import pickle
import random
import os

from berkeley.graphicsDisplay import PacmanGraphics as BerkeleyGraphics
from berkeley.layout import getLayout as get_berkeley_layout
from berkeley.pacman import runGames as run_berkeley_games
from berkeley.textDisplay import NullGraphics as BerkeleyNullGraphics

import agents
import communication as comm

import cliparser


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


## @todo Parse arguments outside class, pass values as arguments for
# constructor.
class Adapter(object):
    ## @todo define pacman-agent choices and ghost-agent choices from agents.py
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
        elif pacman_agent == 'ai':
            self.pacman_class = agents.BehaviorLearningPacmanAgent
        elif pacman_agent == 'eater':
            self.pacman_class = agents.EaterPacmanAgent
        else:
            raise ValueError('Pac-Man agent must be ai, random or eater.')

        if not isinstance(client, comm.ZMQMessengerBase):
            raise ValueError('Invalid client')

        self.pacman = agents.CommunicatingPacmanAgent(client=client)
        log('Created {} #{}.'.format(self.pacman_class.__name__,
                                     self.pacman.agent_id))
        log('Request register for {} #{}.'.format(self.pacman_class.__name__,
                                                  self.pacman.agent_id))
        self.pacman.register_agent('pacman', self.pacman_class)

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
            ghost = agents.CommunicatingGhostAgent(x+1, client=client)
            log('Created {} #{}.'.format(ghost_name, ghost.agent_id))
            log('Request register for {} #{}.'.format(ghost_name,
                                                      ghost.agent_id))
            ghost.register_agent('ghost', self.ghost_class)
            self.ghosts.append(ghost)

        self.all_agents = [self.pacman] + self.ghosts

        # Setup policy file
        self.policy_file = str(policy_file)

        # Setup runs
        self.learn_runs = int(learn_runs)
        if self.learn_runs < 1:
            raise ValueError('Number of learning runs must be at least 1.')

        self.test_runs = int(test_runs)
        if self.test_runs < 1:
            raise ValueError('Number of test runs must be at least 1.')

        self.output_file = str(output_file)

        if graphics:
            self.display = BerkeleyPacmanGraphics(zoom, frameTime=frameTime)
        else:
            self.display = BerkeleyNullGraphics()

        log('Ready')

    def __communicate_state__(self, agent, state):
        msg = agent.create_state_message(state)
        agent.communicate(msg)

    def __get_current_policy__(self, agent):
        msg = comm.RequestPolicyMessage(agent.agent_id)
        reply_msg = agent.communicate(msg)
        return reply_msg.policy

    def __load_policies_from_file__(self, policy_filename):
        policies = {}
        if policy_filename and os.path.isfile(policy_filename):
            log('Loading policies from {}.'.format(policy_filename))
            with open(policy_filename) as f:
                policies = pickle.loads(f.read())
        return policies

    def __load_policy__(self, agent, policy):
        log('Loading {} #{} policy.'.format(type(agent).__name__,
                                            agent.agent_id))

        msg = comm.PolicyMessage(agent_id=agent.agent_id, policy=policy)
        agent.communicate(msg)

    def __log_behavior_count__(self, agent, results):
        msg = comm.RequestBehaviorCountMessage(agent_id=agent.agent_id)
        reply_msg = agent.communicate(msg)

        log('{} behavior count: {}.'.format(type(agent).__name__,
                                            reply_msg.count))

        for behavior, count in reply_msg.count.items():
            if behavior not in results['behavior_count'][agent.agent_id]:
                results['behavior_count'][agent.agent_id][behavior] = []
            results['behavior_count'][agent.agent_id][behavior].append(count)

    def __process_game__(self, policies, results):
        # Start new game
        for agent in self.all_agents:
            agent.start_game(self.layout.width, self.layout.height)

        # Load policies to agents
        if policies:
            for agent in self.all_agents:
                if agent.agent_id in policies:
                    self.__load_policy__(agent, policies[agent.agent_id])

        log('Simulate game')
        simulated_game = run_berkeley_games(self.layout, self.pacman,
                                            self.ghosts, self.display,
                                            NUMBER_OF_BERKELEY_GAMES,
                                            RECORD_BERKELEY_GAMES)[0]

        # Do this so as agents can receive the last reward
        for agent in self.all_agents:
            self.__communicate_state__(agent, simulated_game.state)

        ## @todo this as one list, probably by checking if agent is
        # instance of BehaviorLearningAgent (needs refactoring).

        # Log behavior count
        if self.pacman_class == agents.BehaviorLearningPacmanAgent:
            self.__log_behavior_count__(pacman, results)

        if self.ghost_class == agents.BehaviorLearningGhostAgent:
            for ghost in self.ghosts:
                self.__log_behavior_count__(ghost, results)

        # Log score
        return simulated_game.state.getScore()

    def __save_policies__(self, filename, policies):
        if self.pacman_class == agents.BehaviorLearningPacmanAgent:
            agent_id = pacman.agent_id
            ## @todo keep policy in agent?
            policies[agent_id] = self.__get_current_policy__(self.pacman)

        if self.ghost_class == agents.BehaviorLearningGhostAgent:
            for ghost in self.ghosts:
                agent_id = ghost.agent_id
                policies[agent_id] = self.__get_current_policy__(ghost)

        self.__write_to_file__(filename, policies)

    def __write_to_file__(self, filename, content):
        with open(filename, 'w') as f:
            f.write(pickle.dumps(content))

    def run(self):
        log('Now running')

        results = {'learn_scores': [], 'test_scores': [], 'behavior_count': {}}

        ## @todo this as one list, probably by checking if agent is instance of
        # BehaviorLearningAgent (needs refactoring).
        if self.pacman_class == agents.BehaviorLearningPacmanAgent:
            results['behavior_count'][pacman.agent_id] = {}

        if self.ghost_class == agents.BehaviorLearningGhostAgent:
            for ghost in self.ghosts:
                results['behavior_count'][ghost.agent_id] = {}

        # Load policies from file
        policies = self.__load_policies_from_file__(self.policy_file)

        # Initialize agents
        for agent in self.all_agents:
            agent.init_agent()

        for x in xrange(self.learn_runs):
            log('LEARN game {} (of {})'.format(x+1, self.learn_runs))

            score = self.__process_game__(policies, results)
            results['learn_scores'].append(score)

        for agent in self.all_agents:
            agent.enable_test_mode()

        for x in xrange(self.test_runs):
            log('TEST game {} (of {})'.format(x+1, self.test_runs))

            score = self.__process_game__(policies, results)
            results['test_scores'].append(score)

        if policies:
            self.__save_policies__(self.policy_file, policies)

        log('Learn scores: {}'.format(results['learn_scores']))
        log('Test scores: {}'.format(results['test_scores']))

        self.__write_to_file__(self.output_file, results)

if __name__ == '__main__':
    try:
        adapter = cliparser.get_Adapter()
        adapter.run()
    except KeyboardInterrupt:
        print '\n\nInterrupted execution\n'
