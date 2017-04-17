#  -*- coding: utf-8 -*-

"""Define the agents."""

from __future__ import absolute_import

import logging
import random

from .berkeley.game import Agent as BerkeleyGameAgent, Directions

from . import behaviors
from . import features
from multiagentrl import core
from multiagentrl import exploration
from multiagentrl import learning


# Logging configuration
logger = logging.getLogger(__name__)


# Berkeley simulator actions
GHOST_ACTIONS = [Directions.NORTH, Directions.SOUTH, Directions.EAST,
                 Directions.WEST]
PACMAN_ACTIONS = GHOST_ACTIONS + [Directions.STOP]

# Berkeley simulator indices
PACMAN_INDEX = 0


class PacmanAgent(core.BaseControllerAgent):
    def __init__(self, agent_id):
        assert agent_id == PACMAN_INDEX
        super(PacmanAgent, self).__init__(agent_id)

    def start_game(self):
        pass

    def finish_game(self):
        pass


class GhostAgent(core.BaseControllerAgent):
    def __init__(self, agent_id):
        assert agent_id != PACMAN_INDEX
        super(GhostAgent, self).__init__(agent_id)

    def start_game(self):
        pass

    def finish_game(self):
        pass


class RandomPacmanAgent(PacmanAgent):
    """Agent that randomly selects an action."""
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(RandomPacmanAgent, self).__init__(agent_id)

    def learn(self, state, action, reward):
        pass

    def act(self, state, legal_actions, explore):
        if legal_actions:
            return random.choice(legal_actions)
        else:
            return Directions.STOP


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
        else:
            return Directions.STOP


class EaterPacmanAgent(PacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(EaterPacmanAgent, self).__init__(agent_id)
        self.eat_behavior = behaviors.EatBehavior()

    def learn(self, state, action, reward):
        pass

    def act(self, state, legal_actions, explore):
        if not legal_actions:
            return Directions.STOP

        suggested_action = self.eat_behavior(state, legal_actions)

        if suggested_action in legal_actions:
            return suggested_action
        else:
            return random.choice(legal_actions)


class TDLearningPacmanAgent(PacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids, learning_algorithm,
                 exploration_algorithm):
        super(TDLearningPacmanAgent, self).__init__(agent_id)
        self.game_number = 1
        self.game_step = 1
        self.exploration_rate = 0.1
        self.features = [
            features.XPositionFeature(),
            features.YPositionFeature(),
            features.VisitedPositionFeature(),
        ]
        self.learning = learning_algorithm
        self.exploration = exploration_algorithm
        self.agent_state = None

    def get_policy(self):
        return self.learning.q_values

    def set_policy(self, weights):
        self.learning.q_values = weights

    def start_game(self):
        self.game_step = 1

    def finish_game(self):
        self.game_number += 1

    def _get_state(self, state):
        if not self.agent_state:
            self.agent_state = tuple([
                feature(state) for feature in self.features])
        return self.agent_state

    def learn(self, state, action, reward):
        self.agent_state = self._get_state(state)
        self.learning.learn(self.agent_state, action, reward)

    def act(self, state, legal_actions, explore):
        if not legal_actions:
            return Directions.STOP

        self.agent_state = self._get_state(state)
        action = self.learning.act(self.agent_state)

        if explore and legal_actions:
            action = self.exploration.explore(action, legal_actions)

        self.game_step += 1

        # Reset agent state
        self.agent_state = None

        return self._select_valid_action(action, legal_actions)

    def _select_valid_action(self, action, legal_actions):
        if action in legal_actions:
            return action
        else:
            return random.choice(legal_actions)


class QLearningPacmanAgent(TDLearningPacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(QLearningPacmanAgent, self).__init__(
            agent_id, ally_ids, enemy_ids,
            learning.QLearning(learning_rate=0.3, discount_factor=0.7,
                               actions=PACMAN_ACTIONS),
            exploration.EGreedy(exploration_rate=0.1))


class SARSALearningPacmanAgent(TDLearningPacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(SARSALearningPacmanAgent, self).__init__(
            agent_id, ally_ids, enemy_ids,
            learning.SARSALearning(learning_rate=0.5, discount_factor=0.9,
                                   actions=PACMAN_ACTIONS),
            exploration.EGreedy(exploration_rate=0.1))


class BehaviorLearningPacmanAgent(TDLearningPacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids, learning_algorithm,
                 exploration_algorithm):
        TDLearningPacmanAgent.__init__(
            self, agent_id, ally_ids, enemy_ids, learning_algorithm,
            exploration_algorithm)

        self.num_behavior_steps = 5
        self.previous_behavior = None
        self.K = 1.0  # Learning rate

    def learn(self, state, action, reward):
        self.agent_state = self._get_state(state)

        if self.previous_behavior:
            self.learning.learning_rate = self.K / (self.K + self.game_step)
            self.learning.learn(self.agent_state, self.previous_behavior,
                                reward)

    def act(self, state, legal_actions, explore):
        if not legal_actions:
            return Directions.STOP

        self.agent_state = self._get_state(state)

        # Select a new behavior every `num_behavior_steps` steps.
        if (self.previous_behavior is None or
                self.game_step % self.num_behavior_steps == 0):
            behavior = self.learning.act(self.agent_state)
            self.previous_behavior = behavior
        else:
            behavior = self.previous_behavior

        action = behavior(state, legal_actions)

        if explore and legal_actions:
            action = self.exploration.explore(action, legal_actions)

        self.game_step += 1

        # Reset agent state
        self.agent_state = None

        return self._select_valid_action(action, legal_actions)


class BehaviorRandomPacmanAgent(TDLearningPacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        TDLearningPacmanAgent.__init__(
            self, agent_id, ally_ids, enemy_ids, None, None
        )

        self.behaviors = [
            behaviors.EatBehavior(),
            behaviors.FleeBehavior(),
            behaviors.SeekBehavior(),
            behaviors.PursueBehavior()
        ]
        self.num_behavior_steps = 5
        self.previous_behavior = None

    def learn(self, state, action, reward):
        pass

    def act(self, state, legal_actions, explore):
        if not legal_actions:
            return Directions.STOP

        if (self.previous_behavior is None or
                self.game_step % self.num_behavior_steps == 0):
            behavior = random.choice(self.behaviors)
            self.previous_behavior = behavior
        else:
            behavior = self.previous_behavior

        action = behavior(state, legal_actions)
        self.game_step += 1
        return self._select_valid_action(action, legal_actions)


class BehaviorQLearningPacmanAgent(BehaviorLearningPacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(BehaviorQLearningPacmanAgent, self).__init__(
            agent_id, ally_ids, enemy_ids,
            learning.QLearning(
                learning_rate=0.3, discount_factor=0.7,
                actions=[
                    behaviors.EatBehavior(),
                    behaviors.FleeBehavior(),
                    behaviors.SeekBehavior(),
                    behaviors.PursueBehavior()
                ]),
            exploration.EGreedy(exploration_rate=0.1))


class BayesianBehaviorQLearningPacmanAgent(BehaviorLearningPacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        self.features = [features.FoodDistanceFeature()]
        for enemy_id in enemy_ids:
            self.features.append(features.EnemyDistanceFeature(enemy_id))
        for id_ in [agent_id] + ally_ids + enemy_ids:
            self.features.append(features.FragileAgentFeature(id_))

        self.behaviors = [
            behaviors.EatBehavior(),
            behaviors.FleeBehavior(),
            behaviors.SeekBehavior(),
            behaviors.PursueBehavior()
        ]

        super(BayesianBehaviorQLearningPacmanAgent, self).__init__(
            agent_id, ally_ids, enemy_ids,
            learning.QLearningWithApproximation(
                learning_rate=0.1, discount_factor=0.9,
                actions=self.behaviors,
                features=self.features),
            exploration.EGreedy(exploration_rate=0.1))
