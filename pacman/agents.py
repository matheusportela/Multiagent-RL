#!/usr/bin/env python
#  -*- coding: utf-8 -*-

"""Define the agents."""

import random

from berkeley.game import Agent as BerkeleyGameAgent, Directions

import behaviors
import features
import learning
import messages

import sys
sys.path.insert(1, '..')
from communication import ZMQMessengerBase

__author__ = "Matheus Portela and Guilherme N. Ramos"
__credits__ = ["Matheus Portela", "Guilherme N. Ramos", "Renato Nobre",
               "Pedro Saman"]
__maintainer__ = "Guilherme N. Ramos"
__email__ = "gnramos@unb.br"

# Default noise configuration
# @todo Receive noise via constructor
NOISE = 0

# Actions
GHOST_ACTIONS = [Directions.NORTH, Directions.SOUTH, Directions.EAST,
                 Directions.WEST]
PACMAN_ACTIONS = GHOST_ACTIONS + [Directions.STOP]
FIRST_ACTION = Directions.STOP

# Indices
PACMAN_INDEX = 0

###############################################################################
#                                AdapterAgents                                #
###############################################################################

# @todo properly include communication module from parent folder
import sys
sys.path.insert(0, '..')
from communication import ZMQClient


def log(msg):
    print '[  Client  ] {}'.format(msg)


class ClientAgent(ZMQClient, BerkeleyGameAgent):
    def __init__(self, agent_id, context, connection):
        super(ClientAgent, self).__init__(context, connection)
        BerkeleyGameAgent.__init__(self, agent_id)

        self.previous_action = self._first_action()
        self.test_mode = False

        self.log('instantiated')

    # Communication stuff #####################################################

    def log(self, msg):
        log('{} #{} {}.'.format(self.__class__.__name__, self.agent_id, msg))

    def receive(self):
        raise AttributeError('ClientAgent cannot receive messages.')

    def send(self, msg):
        raise AttributeError('ClientAgent cannot send messages.')

    def communicate(self, msg):
        """Synchronous communication."""
        ZMQClient.send(self, msg)
        return ZMQClient.receive(self)

    # BerkeleyGameAgent stuff #################################################

    @property
    def agent_id(self):
        return self.index  # from BerkeleyGameAgent

    def getAction(self, state):
        """Returns a legal action (from Directions)."""

        reply_msg = self.update(state)
        self.previous_action = reply_msg.action
        return reply_msg.action

    # Other stuff #############################################################

    def _first_action(self):
        raise NotImplementedError('Agent must define an initial action')

    def _noise_error(self):
        # @todo Put this in the right place (when perceiving the environment)
        return random.randrange(-NOISE, NOISE + 1)

    def calculate_reward(self, current_score):
        raise NotImplementedError('Communicating agent must calculate score')

    def create_state_message(self, state):
        agent_positions = {}

        agent_positions[PACMAN_INDEX] = state.getPacmanPosition()[::-1]

        for id_, pos in enumerate(state.getGhostPositions()):
            pos_y = pos[::-1][0] + self._noise_error()
            pos_x = pos[::-1][1] + self._noise_error()
            agent_positions[id_ + 1] = (pos_y, pos_x)

        food_positions = []
        for x, row in enumerate(state.getFood()):
            for y, is_food in enumerate(row):
                if is_food:
                    food_positions.append((y, x))

        fragile_agents = {}
        for id_, s in enumerate(state.data.agentStates):
            fragile_agents[id_] = 1.0 if s.scaredTimer > 0 else 0.0

        wall_positions = []
        for x, row in enumerate(state.getWalls()):
            for y, is_wall in enumerate(row):
                if is_wall:
                    wall_positions.append((y, x))

        reward = self.calculate_reward(state.getScore())
        self.previous_score = state.getScore()

        msg = messages.StateMessage(
            agent_id=self.agent_id,
            agent_positions=agent_positions,
            food_positions=food_positions,
            fragile_agents=fragile_agents,
            wall_positions=wall_positions,
            legal_actions=state.getLegalActions(self.agent_id),
            reward=reward,
            executed_action=self.previous_action,
            test_mode=self.test_mode)

        return msg

    def enable_learn_mode(self):
        self.test_mode = False

    def enable_test_mode(self):
        self.test_mode = True

    # Messaging ###############################################################
    def get_behavior_count(self):
        msg = messages.RequestBehaviorCountMessage(self.agent_id)
        reply_msg = self.communicate(msg)
        self.log(' got behavior count: {}'.format(reply_msg.count))
        return reply_msg.count

    def get_policy(self):
        msg = messages.RequestPolicyMessage(self.agent_id)
        reply_msg = self.communicate(msg)
        self.log(' got policy')
        return reply_msg.policy

    def initialize(self):
        self.log('requested initialization')
        msg = messages.RequestInitializationMessage(self.agent_id)
        return self.communicate(msg)

    def load_policy(self, policy):
        self.log('sent policy')
        msg = messages.PolicyMessage(policy)
        return self.communicate(msg)

    def register(self, agent_team, agent_class):
        self.log('requested register {}/{}'.format(agent_team,
                                                   agent_class.__name__))
        msg = messages.RequestRegisterMessage(
            self.agent_id, agent_team, agent_class)
        return self.communicate(msg)

    def start_game(self, layout):
        self.log('requested game start')
        self.previous_score = 0
        self.previous_action = self._first_action()
        msg = messages.RequestGameStartMessage(
            agent_id=self.agent_id,
            map_width=layout.width,
            map_height=layout.height)
        return self.communicate(msg)

    def update(self, state):
        msg = self.create_state_message(state)
        return self.communicate(msg)


class PacmanAdapterAgent(ClientAgent):
    def __init__(self, context, connection):
        super(PacmanAdapterAgent, self).__init__(PACMAN_INDEX, context,
                                                 connection)

    def _first_action(self):
        self.previous_action = Directions.NORTH

    # @todo is this ever used?
    # def act_when_invalid(self, state):
    #     return Directions.STOP

    def calculate_reward(self, current_score):
        return current_score - self.previous_score


class GhostAdapterAgent(ClientAgent):
    def __init__(self, agent_id, context, connection):
        super(GhostAdapterAgent, self).__init__(agent_id, context, connection)

    def _first_action(self):
        self.previous_action = Directions.NORTH
        # self.actions = GHOST_ACTIONS

    # @todo is this ever used?
    # def act_when_invalid(self, state):
    #     return random.choice(state.getLegalActions(self.agent_id))

    def calculate_reward(self, current_score):
        return self.previous_score - current_score

###############################################################################
#                              ControllerAgents                               #
###############################################################################


class ControllerAgent(object):
    """Autonomous agent for game controller."""
    def __init__(self, agent_id):
        self.agent_id = agent_id

    def choose_action(self, state, action, reward, legal_actions, explore):
        """Return an action to be executed by the agent.

        Args:
            state: Current game state.
            action: Last executed action.
            reward: Reward for the previous action.
            legal_actions: List of currently allowed actions.
            explore: Boolean whether agent is allowed to explore.

        Returns:
            A Direction for the agent to follow (NORTH, SOUTH, EAST, WEST or
            STOP).
        """
        raise NotImplementedError('ControllerAgent must implement '
                                  'choose_action.')


class PacmanAgent(ControllerAgent):
    def __init__(self, agent_id):
        super(PacmanAgent, self).__init__(agent_id)


class GhostAgent(ControllerAgent):
    def __init__(self, agent_id):
        super(GhostAgent, self).__init__(agent_id)


class RandomPacmanAgent(PacmanAgent):
    """Agent that randomly selects an action."""
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(RandomPacmanAgent, self).__init__(agent_id)

    def choose_action(self, state, action, reward, legal_actions, explore):
        if legal_actions:
            return random.choice(legal_actions)


class RandomPacmanAgentTwo(PacmanAgent):
    """Agent that after choosing a random direction will follow that direction
    until it reaches a wall or have more than three possible moves. In these
    case, continue to follow the previous directions have twice the chance of
    happening then the other possible movements"""
    def choose_action(self, state, action, reward, legal_actions, explore):

        if action == 'Stop' or action not in legal_actions:
            if 'Stop' in legal_actions:
                legal_actions.remove('Stop')
            if len(legal_actions) > 0:
                return random.choice(legal_actions)
        else:
            if len(legal_actions) > 3:
                if len(legal_actions) == 4:
                    number = random.choice([1, 2, 3, 4, 5])
                else:
                    number = random.choice([1, 2, 3, 4, 5, 6])
                if number == 1 or number == 2:
                    return action
                else:
                    aux = 3
                    legal_actions.remove(action)
                    for possible_action in legal_actions:
                        if number == aux:
                            return possible_action
                        else:
                            aux += 1
                    else:
                        return random.choice(legal_actions)
            else:
                return action


class RandomGhostAgent(GhostAgent):
    """Agent that randomly selects an action."""
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(RandomGhost, self).__init__(agent_id)

    def choose_action(self, state, action, reward, legal_actions, explore):
        if legal_actions:
            return random.choice(legal_actions)


class EaterPacmanAgent(PacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(EaterPacmanAgent, self).__init__(agent_id)
        self.eat_behavior = behaviors.EatBehavior()

    def choose_action(self, state, action, reward, legal_actions, test):
        suggested_action = self.eat_behavior(state, legal_actions)

        if suggested_action in legal_actions:
            return suggested_action
        elif legal_actions == []:
            return Directions.STOP
        else:
            return random.choice(legal_actions)


class BehaviorLearningPacmanAgent(PacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(BehaviorLearningPacmanAgent, self).__init__(agent_id)
        self.features = [features.FoodDistanceFeature()]
        for enemy_id in enemy_ids:
            self.features.append(features.EnemyDistanceFeature(enemy_id))
        for id_ in [agent_id] + ally_ids + enemy_ids:
            self.features.append(features.FragileAgentFeature(id_))

        self.behaviors = [behaviors.EatBehavior(),
                          behaviors.FleeBehavior(),
                          behaviors.SeekBehavior(),
                          behaviors.PursueBehavior()]

        self.K = 1.0  # Learning rate
        self.exploration_rate = 0.1

        QLearning = learning.QLearningWithApproximation
        self.learning = QLearning(learning_rate=0.1, discount_factor=0.9,
                                  actions=self.behaviors,
                                  features=self.features,
                                  exploration_rate=self.exploration_rate)
        self.previous_behavior = self.behaviors[0]
        self.behavior_count = {}
        self.reset_behavior_count()

        self.test_mode = False

    def reset_behavior_count(self):
        for behavior in self.behaviors:
            self.behavior_count[str(behavior)] = 0

    def get_policy(self):
        return self.learning.get_weights()

    def set_policy(self, weights):
        self.learning.set_weights(weights)

    def choose_action(self, state, action, reward, legal_actions, test):
        if test:
            self.enable_test_mode()
        else:
            self.enable_learn_mode()

        if not self.test_mode:
            self.learning.learning_rate = self.K / (self.K + state.iteration)
            self.learning.learn(state, self.previous_behavior, reward)

        behavior = self.learning.act(state)
        self.previous_behavior = behavior
        suggested_action = behavior(state, legal_actions)

        self.behavior_count[str(behavior)] += 1

        if suggested_action in legal_actions:
            return suggested_action
        elif legal_actions == []:
            return Directions.STOP
        else:
            return random.choice(legal_actions)

    def enable_learn_mode(self):
        self.test_mode = False
        self.learning.exploration_rate = self.exploration_rate

    def enable_test_mode(self):
        self.test_mode = True
        self.learning.exploration_rate = 0


class BehaviorLearningGhostAgent(GhostAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(BehaviorLearningGhostAgent, self).__init__(agent_id)
        self.features = [features.FoodDistanceFeature()]
        for enemy_id in enemy_ids:
            self.features.append(features.EnemyDistanceFeature(enemy_id))
        for id_ in [agent_id] + ally_ids + enemy_ids:
            self.features.append(features.FragileAgentFeature(id_))

        self.behaviors = [behaviors.FleeBehavior(),
                          behaviors.SeekBehavior(),
                          behaviors.PursueBehavior()]

        self.K = 1.0  # Learning rate
        self.exploration_rate = 0.1
        QLearning = learning.QLearningWithApproximation
        self.learning = QLearning(learning_rate=0.1, discount_factor=0.9,
                                  actions=self.behaviors,
                                  features=self.features,
                                  exploration_rate=self.exploration_rate)
        self.previous_behavior = self.behaviors[0]
        self.behavior_count = {}
        self.reset_behavior_count()

        self.test_mode = False

    def reset_behavior_count(self):
        for behavior in self.behaviors:
            self.behavior_count[str(behavior)] = 0

    def get_policy(self):
        return self.learning.get_weights()

    def set_policy(self, weights):
        self.learning.set_weights(weights)

    def choose_action(self, state, action, reward, legal_actions, test):
        if test:
            self.enable_test_mode()
        else:
            self.enable_learn_mode()

        if not self.test_mode:
            self.learning.learning_rate = self.K / (self.K + state.iteration)
            self.learning.learn(state, self.previous_behavior, reward)

        behavior = self.learning.act(state)
        self.previous_behavior = behavior
        suggested_action = behavior(state, legal_actions)

        self.behavior_count[str(behavior)] += 1

        if suggested_action in legal_actions:
            return suggested_action
        elif legal_actions == []:
            return Directions.STOP
        else:
            return random.choice(legal_actions)

    def enable_learn_mode(self):
        self.test_mode = False
        self.learning.exploration_rate = self.exploration_rate

    def enable_test_mode(self):
        self.test_mode = True
        self.learning.exploration_rate = 0
