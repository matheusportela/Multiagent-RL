#!/usr/bin/env python
import math
import random

import behaviors
import features
import learning


class PacmanAgent(object):
    """Pacman agent abstract class.

    Attributes:
        index: Pacman agent index for game referral.
    """
    def __init__(self, agent_id, ally_ids, enemy_ids):
        self.agent_id = agent_id
        self.actions = ['North', 'South', 'East', 'West', 'Stop']

    def choose_action(self, state, action, reward, legal_actions, explore):
        """Select an action to be executed by the agent.

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
        raise NotImplementedError('Pacman agent must contain a choose_action method'
            'to select an action for the current game state.')

    def save_policy(self, filename):
        """Save the learned policy into filename.

        Args:
            filename: File which stores the policy data.
        """
        raise NotImplementedError('Pacman agent must be able to save its learned'
            'policy')

    def load_policy(self, filename):
        """Save the learned policy into filename.

        Args:
            filename: File which stores the policy data.
        """
        raise NotImplementedError('Pacman agent must be able to save its learned'
            'policy')


class GhostAgent(object):
    """Ghost agent abstract class.

    Attributes:
        index: Ghost agent index for game referral.
    """
    def __init__(self, agent_id, ally_ids, enemy_ids):
        self.agent_id = agent_id
        self.actions = ['North', 'South', 'East', 'West']

    def choose_action(self, state, action, reward, legal_actions, explore):
        """Select an action to be executed by the agent.

        Ghosts can only select new actions at intersections or dead ends.

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
        raise NotImplementedError('Ghost agent must contain a choose_action method'
            'to select an action for the current game state.')


class RandomPacmanAgent(PacmanAgent):
    """Agent that randomly selects an action."""
    def choose_action(self, state, action, reward, legal_actions, explore):
        if len(legal_actions) > 0:
            return random.choice(legal_actions)


class RandomGhostAgent(GhostAgent):
    """Agent that randomly selects an action."""
    def choose_action(self, state, action, reward, legal_actions, explore):
        if len(legal_actions) > 0:
            return random.choice(legal_actions)


class QLearningAgent(PacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(QLearningAgent, self).__init__(agent_id, ally_ids, enemy_ids)
        self.exploration_rate = 0.1
        self.learning = learning.QLearning(learning_rate=0.1, discount_factor=0.9,
            actions=self.actions)

    def choose_action(self, state, action, reward, legal_actions, explore):
        self.learning.learn(state, action, reward)
        suggested_action = self.learning.act(state, legal_actions)

        if random.random() < self.exploration_rate:
            return random.choice(legal_actions)
        else:
            return suggested_action


class QLearningWithApproximationAgent(PacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(QLearningWithApproximationAgent, self).__init__(agent_id, ally_ids, enemy_ids)
        self.features = [self.feature_ghost_distance, self.feature_food_distance]
        self.exploration_rate = 0.1
        self.learning = learning.QLearningWithApproximation(learning_rate=0.1,
            discount_factor=0.9, actions=self.actions, features=self.features)

    def _find_closest_distance(self, agent_position, position_list):
        closest_distance = float('inf')

        for position in position_list:
            distance = math.sqrt((agent_position[0] - position[0])**2 + (agent_position[1] - position[1])**2)
            if distance < closest_distance:
                closest_distance = distance

        return closest_distance

    def feature_ghost_distance(self, state, action):
        pacman_position = state[0]
        ghost_positions = state[1]
        return self._find_closest_distance(pacman_position, ghost_positions)

    def feature_food_distance(self, state, action):
        pacman_position = state[0]
        food_positions = state[2]
        return self._find_closest_distance(pacman_position, food_positions)

    def choose_action(self, state, action, reward, legal_actions, explore):
        self.learning.learn(state, action, reward)
        suggested_action = self.learning.act(state, legal_actions)

        if random.random() < self.exploration_rate:
            return random.choice(legal_actions)
        else:
            return suggested_action


class EaterPacmanAgent(PacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(EaterPacmanAgent, self).__init__(agent_id, ally_ids, enemy_ids)
        self.eat_behavior = behaviors.EatBehavior()

    def choose_action(self, state, action, reward, legal_actions, test):
        suggested_action = self.eat_behavior(state, legal_actions)

        if suggested_action in legal_actions:
            return suggested_action
        elif legal_actions == []:
            return 'Stop'
        else:
            return random.choice(legal_actions)


class BehaviorLearningPacmanAgent(PacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(BehaviorLearningPacmanAgent, self).__init__(agent_id, ally_ids, enemy_ids)
        self.features = [features.FoodDistanceFeature()]
        for enemy_id in enemy_ids:
            self.features.append(features.EnemyDistanceFeature(enemy_id))
        for id_ in [agent_id] + ally_ids + enemy_ids:
            self.features.append(features.FragileAgentFeature(id_))

        self.behaviors = [behaviors.EatBehavior(), behaviors.FleeBehavior(),
            behaviors.SeekBehavior(), behaviors.PursueBehavior()]

        self.K = 1.0 # Learning rate
        self.exploration_rate = 0.1
        self.learning = learning.QLearningWithApproximation(learning_rate=0.1,
            discount_factor=0.9, actions=self.behaviors, features=self.features,
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
            self.learning.learning_rate = self.K/(self.K + state.iteration)
            self.learning.learn(state, self.previous_behavior, reward)

        behavior = self.learning.act(state)
        self.previous_behavior = behavior
        suggested_action = behavior(state, legal_actions)

        self.behavior_count[str(behavior)] += 1

        if suggested_action in legal_actions:
            return suggested_action
        elif legal_actions == []:
            return 'Stop'
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
        super(BehaviorLearningGhostAgent, self).__init__(agent_id, ally_ids, enemy_ids)
        self.features = [features.FoodDistanceFeature()]
        for enemy_id in enemy_ids:
            self.features.append(features.EnemyDistanceFeature(enemy_id))
        for id_ in [agent_id] + ally_ids + enemy_ids:
            self.features.append(features.FragileAgentFeature(id_))

        self.behaviors = [behaviors.FleeBehavior(), behaviors.SeekBehavior(),
            behaviors.PursueBehavior()]

        self.K = 1.0 # Learning rate
        self.exploration_rate = 0.1
        self.learning = learning.QLearningWithApproximation(learning_rate=0.1,
            discount_factor=0.9, actions=self.behaviors, features=self.features,
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
            self.learning.learning_rate = self.K/(self.K + state.iteration)
            self.learning.learn(state, self.previous_behavior, reward)

        behavior = self.learning.act(state)
        self.previous_behavior = behavior
        suggested_action = behavior(state, legal_actions)

        self.behavior_count[str(behavior)] += 1

        if suggested_action in legal_actions:
            return suggested_action
        elif legal_actions == []:
            return 'Stop'
        else:
            return random.choice(legal_actions)

    def enable_learn_mode(self):
        self.test_mode = False
        self.learning.exploration_rate = self.exploration_rate

    def enable_test_mode(self):
        self.test_mode = True
        self.learning.exploration_rate = 0