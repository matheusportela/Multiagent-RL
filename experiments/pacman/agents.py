#!/usr/bin/env python
#  -*- coding: utf-8 -*-

"""Define the agents."""

import random

from berkeley.game import Agent as BerkeleyGameAgent, Directions

import behaviors
import features

# @todo properly include multiagentrl without relative imports
import sys
sys.path.insert(0, '..')
from multiagentrl import core
from multiagentrl import exploration
from multiagentrl import learning

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


class PacmanAgent(core.BaseControllerAgent):
    def __init__(self, agent_id):
        super(PacmanAgent, self).__init__(agent_id)


class GhostAgent(core.BaseControllerAgent):
    def __init__(self, agent_id):
        super(GhostAgent, self).__init__(agent_id)


class RandomPacmanAgent(PacmanAgent):
    """Agent that randomly selects an action."""
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(RandomPacmanAgent, self).__init__(agent_id)

    def learn(self, state, action, reward):
        pass

    def act(self, state, legal_actions, explore):
        if legal_actions:
            return random.choice(legal_actions)


class RandomPacmanAgentTwo(PacmanAgent):
    """Agent that after choosing a random direction will follow that direction
    until it reaches a wall or have more than three possible moves. In these
    case, continue to follow the previous directions have twice the chance of
    happening then the other possible movements"""
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(RandomPacmanAgentTwo, self).__init__(agent_id)
        self.last_action = None

    def learn(self, state, action, reward):
        self.last_action = action

    def act(self, state, legal_actions, explore):
        if self.last_action == 'Stop' or self.last_action not in legal_actions:
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
                    return self.last_action
                else:
                    aux = 3
                    legal_actions.remove(self.last_action)
                    for possible_action in legal_actions:
                        if number == aux:
                            return possible_action
                        else:
                            aux += 1
                    else:
                        return random.choice(legal_actions)
            else:
                return self.last_action


class RandomGhostAgent(GhostAgent):
    """Agent that randomly selects an action."""
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(RandomGhostAgent, self).__init__(agent_id)

    def learn(self, state, action, reward):
        pass

    def act(self, state, legal_actions, explore):
        if legal_actions:
            return random.choice(legal_actions)


class EaterPacmanAgent(PacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(EaterPacmanAgent, self).__init__(agent_id)
        self.eat_behavior = behaviors.EatBehavior()

    def learn(self, state, action, reward):
        pass

    def act(self, state, legal_actions, explore):
        suggested_action = self.eat_behavior(state, legal_actions)

        if legal_actions == []:
            return Directions.STOP
        elif suggested_action in legal_actions:
            return suggested_action
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

        self.learning = learning.QLearningWithApproximation(
            learning_rate=0.1, discount_factor=0.9, actions=self.behaviors,
            features=self.features)
        self.exploration = exploration.EGreedy(
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

    def learn(self, state, action, reward):
        self.learning.learning_rate = self.K / (self.K + state.iteration)
        self.learning.learn(state, self.previous_behavior, reward)

    def act(self, state, legal_actions, explore):
        behavior = self.learning.act(state)
        self.previous_behavior = behavior
        suggested_action = behavior(state, legal_actions)

        if explore:
            suggested_action = self.exploration.explore(
                suggested_action, legal_actions)

        self.behavior_count[str(behavior)] += 1

        if legal_actions == []:
            return Directions.STOP
        elif suggested_action in legal_actions:
            return suggested_action
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
        self.learning = learning.QLearningWithApproximation(
            learning_rate=0.1, discount_factor=0.9, actions=self.behaviors,
            features=self.features)
        self.exploration = exploration.EGreedy(
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

    def learn(self, state, action, reward):
        self.learning.learning_rate = self.K / (self.K + state.iteration)
        self.learning.learn(state, self.previous_behavior, reward)

    def act(self, state, legal_actions, explore):
        behavior = self.learning.act(state)
        self.previous_behavior = behavior
        suggested_action = behavior(state, legal_actions)

        if explore:
            suggested_action = self.exploration.explore(
                suggested_action, legal_actions)

        self.behavior_count[str(behavior)] += 1

        if legal_actions == []:
            return Directions.STOP
        elif suggested_action in legal_actions:
            return suggested_action
        else:
            return random.choice(legal_actions)

    def enable_learn_mode(self):
        self.test_mode = False
        self.learning.exploration_rate = self.exploration_rate

    def enable_test_mode(self):
        self.test_mode = True
        self.learning.exploration_rate = 0
