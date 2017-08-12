#  -*- coding: utf-8 -*-

"""Collection of reinforcement learning algorithms.

This module contains classes implementing multiple reinforcement learning
algorithms. These classes are agnostic to their usage, containing no
information specific to environments where they will be applied to.

In order to achieve that, all classes must inherit from BaseLearningAlgorithm
and implement its virtual methods.
"""

import copy
import logging
import random

# Logging configuration
logger = logging.getLogger(__name__)


class BaseLearningAlgorithm(object):
    def learn(self, state, action, reward):
        """Learn from experience.

        Learn by applying the reward received when transitioning from the
        current state to the new one by executing an action.

        Parameters:
        state -- Agent state after executing the action.
        action -- Executed action.
        reward -- Reward received after executing the action.
        """
        raise NotImplementedError('%s does not implement "learn" method'
                                  % str(type(self)))

    def act(self, state):
        """Select an action for the given state.

        By exploiting learned model, the algorithm selects the best action to
        be executed by the agent.

        Parameters:
        state -- Agent state to select an action.
        """
        raise NotImplementedError('%s does not implement "act" method'
                                  % str(type(self)))


class TDLearning(BaseLearningAlgorithm):
    divergence_threshold = 1e6

    def __init__(self, learning_rate=0.1, discount_factor=0.9, actions=None):
        self.previous_state = None
        self.previous_action = None
        self.q_values = {}
        self.learning_rate = learning_rate
        self.discount_factor = discount_factor

        if actions:
            self.actions = actions
        else:
            self.actions = []

    def __str__(self):
        """Generates Q-values string representation."""
        results = []

        # Print table header
        results.append('Q-values ')
        for action in self.actions:
            results.append('{0: ^9}    '.format(str(action)))
        results.append('\n')

        # Print Q-values
        for state in sorted(self.q_values):
            if not any(self.q_values[state].values()):
                continue

            results.append('{0: >4}:    '.format(str(state)))
            for action in self.q_values[state]:
                q_value = self.q_values[state][action]

                if q_value == 0.0:
                    results.append('{0: >9}    '.format('0'))
                else:
                    results.append('{0:+f}    '.format(q_value))
            results.append('\n')
        return ''.join(results)

    def _update_previous_state(self, state):
        """Update learning previous state.

        Parameters:
        state -- State to which the learning algorithm is going.
        """
        self.previous_state = copy.deepcopy(state)

    def _update_previous_action(self, action):
        """Update learning previous action.

        Parameters:
        action -- Action to which the learning algorithm is going.
        """
        self.previous_action = copy.deepcopy(action)

    def _initialize_unknown_state(self, state):
        """Initialize Q-values for states that were not previously seen.

        Parameters:
        state -- Environment state.
        """
        if state not in self.q_values:
            self.q_values[state] = {}
            for action_ in self.actions:
                self.q_values[state][action_] = 0.0

    def _get_q_value(self, state, action):
        """Get the current estimated value for the state-action pair.

        Parameters:
        state -- Environment state.
        action -- Agent action.
        """
        self._initialize_unknown_state(state)
        return self.q_values[state][action]

    def _set_q_value(self, state, action, value):
        """Set a new estimated value for the state-action pair.

        Parameters:
        state -- Environment state.
        action -- Agent action.
        value -- New estimated value.
        """
        if abs(value) > self.divergence_threshold:
            logger.warn('Learning seems to be diverging')

        self.q_values[state][action] = value

    def _get_max_action_from_list(self, state):
        """Get the action with maximum estimated value from the given list of
        actions.

        state -- Environment state.
        """
        actions = self.q_values[state]
        values = [self.q_values[state][action] for action in actions]
        max_value = max(values)
        max_actions = [action for action in actions
                       if self.q_values[state][action] == max_value]

        return random.choice(max_actions)

    def _get_max_action(self, state):
        """Get the action with maximum estimated value.

        Parameters:
        state -- Environment state.
        """
        self._initialize_unknown_state(state)
        return self._get_max_action_from_list(state)

    def _get_max_q_value(self, state):
        max_action = self._get_max_action(state)
        return self.q_values[state][max_action]

    def act(self, state):
        """Select the best legal action for the given state.

        Parameters:
        state -- Agent state to select an action.
        """
        return self._get_max_action(state)


class SARSALearning(TDLearning):
    def learn(self, state, action, reward):
        """Learn by updating the (state, action) reward.

        Learn by applying the reward received when transitioning from the
        current state to the new one by executing an action.

        Parameters:
        state -- Agent state after executing the action.
        action -- Executed action.
        reward -- Reward received after executing the action.
        """
        logger.debug('Previous state: {}'.format(self.previous_state))
        logger.debug('Previous action: {}'.format(self.previous_action))
        logger.debug('Current state: {}'.format(state))
        logger.debug('Current action: {}'.format(action))
        logger.debug('Reward: {}'.format(reward))

        if self.previous_state and self.previous_action:
            old_value = self._get_q_value(
                self.previous_state, self.previous_action)
            next_value = self._get_q_value(state, action)
            new_value = (old_value + self.learning_rate*(
                reward + self.discount_factor*next_value - old_value))
            self._set_q_value(
                self.previous_state, self.previous_action, new_value)
        self._update_previous_state(state)
        self._update_previous_action(action)

        logger.debug('Q-values:\n{}'.format(self))


class QLearning(TDLearning):
    """Q-learning algorithm implementation.

    Q-learning is a model free reinforcement learning algorithm that tries and
    learning state values and chooses actions that maximize the expected
    discounted reward for the current state.
    """
    def learn(self, state, action, reward):
        """Learn by updating the (state, action) reward.

        Learn by applying the reward received when transitioning from the
        current state to the new one by executing an action.

        Parameters:
        state -- Agent state after executing the action.
        action -- Executed action.
        reward -- Reward received after executing the action.
        """
        logger.debug('Previous state: {}'.format(self.previous_state))
        logger.debug('Current state: {}'.format(state))
        logger.debug('Action: {}'.format(action))
        logger.debug('Reward: {}'.format(reward))

        if self.previous_state:
            old_value = self._get_q_value(self.previous_state, action)
            next_expected_value = self._get_max_q_value(state)
            new_value = (old_value + self.learning_rate*(
                reward + self.discount_factor*next_expected_value - old_value))
            self._set_q_value(self.previous_state, action, new_value)
        self._update_previous_state(state)

        logger.debug('Q-values:\n{}'.format(self))


class QLearningWithApproximation(BaseLearningAlgorithm):
    divergence_threshold = 1e6

    def __init__(self, actions=None, features=None, learning_rate=0.1,
                 discount_factor=0.9):
        super(QLearningWithApproximation, self).__init__()
        self.actions = actions
        self.learning_rate = learning_rate
        self.discount_factor = discount_factor
        self.previous_state = None
        self.weights = {}

        if features:
            self.features = features

    @property
    def features(self):
        return self._features

    @features.setter
    def features(self, features):
        self._features = features
        self._init_weights()

    def _init_weights(self):
        for action in self.actions:
            self.weights[str(action)] = [
                random.random() for _ in self.features]

    def get_weights(self):
        return self.weights

    def set_weights(self, weights):
        self.weights = weights

    def _get_q_value(self, state, action):
        q_value = 0

        for weight, feature in zip(self.weights[str(action)], self.features):
            q_value += weight * feature(state)

        return q_value

    def _get_max_action(self, state):
        """Get the action with maximum estimated value from the given list of
        actions.

        state -- Environment state.
        """
        values = [self._get_q_value(state, action) for action in self.actions]
        max_value = max(values)
        max_actions = [action for action, value in zip(self.actions, values)
                       if value == max_value]

        return random.choice(max_actions)

    def _get_max_q_value(self, state):
        action = self._get_max_action(state)
        return self._get_q_value(state, action)

    def _update_weights(self, action, delta):
        self.weights[str(action)] = [
            weight + self.learning_rate*delta*feature(self.previous_state)
            for weight, feature in zip(self.weights[str(action)],
                                       self.features)]

        for weight in self.weights[str(action)]:
            if abs(weight) > QLearningWithApproximation.divergence_threshold:
                logger.warn(
                    'Q-learning with approximation seems to be diverging')

    def learn(self, state, action, reward):
        if self.previous_state:
            delta = (reward + self.discount_factor *
                     self._get_max_q_value(state) -
                     self._get_q_value(self.previous_state, action))

            self._update_weights(action, delta)

        self.previous_state = copy.deepcopy(state)

    def act(self, state):
        return self._get_max_action(state)
