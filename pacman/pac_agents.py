#  -*- coding: utf-8 -*-
##    @package pac_agents.py
#      @author Guilherme N. Ramos (gnramos@unb.br)
#
# Implements the agent classes for simulations with the Berkeley Pac-Man Game.


from random import choice as random_choice

from berkeley.game import Agent as BerkeleyGameAgent

from pac_utils import Logger


# Pac-Man game configuration
PACMAN_INDEX = 0  # As established in berkeley.game


class AgentBase(object, BerkeleyGameAgent):
    '''Base class for agents in the Pac-Man game.

    The agent will receive a berkeley.pacman.GameState and must implement
    the getAction method to return an action from
    berkeley.game.Directions.{North, South, East, West, Stop}.
    '''

    def __init__(self, index):
        '''Constructor.

        Keyword arguments:
        index -- an integer uniquely identifying the agent
        '''
        BerkeleyGameAgent.__init__(self, index)

        self.logger = Logger(self)
        self.logger.log('Instantiated')

    def init(self):
        '''Initializes the agent.'''
        raise NotImplementedError('Agents must be initialized.')

    def setup(self):
        '''Prepares the agent for a series of games.'''
        raise NotImplementedError('Agents must setup itself for new game.')

    def cleanup(self):
        '''Housekeeping after a series of games.'''
        raise NotImplementedError('Agents must be cleaned up after game.')

    def __str__(self):
        return '{} #{}'.format(Logger.__str__(self), self.index)


class RandomAgent(AgentBase):
    '''Acts randomly.'''
    def __init__(self, index):
        super(RandomAgent, self).__init__(index)

    def getAction(self, state):
        '''Returns a random [valid] action.

        Keyword arguments:
        state -- the current game state
        '''
        return random_choice(state.getLegalActions(self.index))

    def setup(self):
        self.logger.log('setup')

    def cleanup(self):
        self.logger.log('cleanup')


class BehaviorInterface():
    def get_behavior_count(self):
        return {}


class LearningInterface():
    def update(self, state):
        self.logger.log('update')

    def enable_test_mode(self):
        self.logger.log('enable_test_mode')


class QLearningInterface(LearningInterface):
    def load(self, policy):
        pass

    def get_policy(self):
        return None


class RandomLearningAgent(RandomAgent, QLearningInterface, BehaviorInterface):
    def __init__(self, index):
        super(RandomLearningAgent, self).__init__(index)


# class CommunicationClient():
#     def __init__(self, context, connection):
#         self.client = ZMQClient(context, connection)

#     def communicate(self, msg):
#         '''Synchronous communication.'''
#         self.client.send(msg)
#         return self.receive()
