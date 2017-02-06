"""Collection of reinforcement learning algorithms"""

from __future__ import division
import copy
import random


class LearningAlgorithm(object):
    def learn(self, state, action, reward):
        """Learn from experience.

        Learn by applying the reward received when transitioning from the
        current state to the new one by executing an action.

        Parameters:
        state -- Agent state after executing the action.
        action -- Executed action.
        reward -- Reward received after executing the action.
        """
        raise (NotImplementedError,
            '%s does not implement "learn" method' % str(type(self)))

    def act(self, state):
        """Select an action for the given state.

        By exploiting learned model, the algorithm selects the best action to be
        executed by the agent.

        Parameters:
        state -- Agent state to select an action.
        """
        raise (NotImplementedError,
            '%s does not implement "act" method' % str(type(self)))


class QLearning(LearningAlgorithm):
    """Q-learning algorithm implementation.

    Q-learning is a model free reinforcement learning algorithm that tries and
    learning state values and chooses actions that maximize the expected
    discounted reward for the current state.

    Instance variables:
    previous_state -- State in which the algorithm currently is.
    q_values -- Storage for (state, action) pair estimated values.
    learning_rate -- Value in [0, 1] interval that determines how much of the
        new information overrides the previous value. Deterministic scenarios
        may have optimal results with learning rate of 1, which means the new
        information completely replaces the old one.
    discount_factor -- Value in [0, 1) interval that determines the importance
        of future rewards. 0 makes the agent myopic and greedy, trying to
        achieve higher rewards in the next step. Closer to 1 makes the agent
        maximize long-term rewards. Although values of 1 and higher are possible,
        it may make the expected discounted reward infinite or divergent.
    """

    def __init__(self, initial_state=0, learning_rate=1, discount_factor=1,
        actions=None):
        """Constructor.

        Parameters:
        initial_state -- State where the algorithm begins.
        num_states -- Number of states to be represented.
        num_actions -- Number of actions to be represented.
        """
        super(QLearning, self).__init__()
        self.previous_state = initial_state
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
        results.append('Q-values\n')
        for state in self.q_values:
            results.append(str(state))
            for action in self.q_values[state]:
                results.append(str(self.q_values[state][action]))
                results.append('\t')
            results.append('\n')
        return ''.join(results)

    def update_state(self, state):
        """Update Q Learning current state.

        Parameters:
        state -- State to which the learning algorithm is going.
        """
        self.previous_state = copy.deepcopy(state)

    def initialize_unknown_state(self, state):
        """Initialize Q-values for states that were not previously seen.

        Parameters:
        state -- Environment state.
        """
        if state not in self.q_values:
            self.q_values[state] = {}
            for action_ in self.actions:
                self.q_values[state][action_] = 0.0

    def get_q_value(self, state, action):
        """Get the current estimated value for the state-action pair.

        Parameters:
        state -- Environment state.
        action -- Agent action.
        """
        self.initialize_unknown_state(state)
        return self.q_values[state][action]

    def set_q_value(self, state, action, value):
        """Set a new estimated value for the state-action pair.

        Parameters:
        state -- Environment state.
        action -- Agent action.
        value -- New estimated value.
        """
        self.q_values[state][action] = value

    def _get_max_action_from_list(self, state, action_list):
        """Get the action with maximum estimated value from the given list of
        actions.

        state -- Environment state.
        action_list -- Actions to be evaluated.
        """
        actions = filter(lambda a: a in action_list, self.q_values[state])
        values = [self.q_values[state][action] for action in actions]
        max_value = max(values)
        max_actions = [action
            for action in actions if self.q_values[state][action] == max_value]

        return random.choice(max_actions)

    def get_max_action(self, state):
        """Get the action with maximum estimated value.

        Parameters:
        state -- Environment state.
        """
        self.initialize_unknown_state(state)
        return self._get_max_action_from_list(state, self.actions)

    def get_max_q_value(self, state):
        max_action = self.get_max_action(state)
        return self.q_values[state][max_action]

    def learn(self, state, action, reward):
        """Learn by updating the (state, action) reward.

        Learn by applying the reward received when transitioning from the
        current state to the new one by executing an action.

        Parameters:
        state -- Agent state after executing the action.
        action -- Executed action.
        reward -- Reward received after executing the action.
        """
        old_value = self.get_q_value(self.previous_state, action)
        next_expected_value = self.get_max_q_value(state)
        new_value = (old_value + self.learning_rate*(reward + self.discount_factor*next_expected_value - old_value))
        self.set_q_value(self.previous_state, action, new_value)
        self.update_state(state)

    def act(self, state, legal_actions):
        """Select the best legal action for the given state.

        Parameters:
        state -- Agent state to select an action.
        legal_actions -- Actions allowed in the current state.
        """
        return self._get_max_action_from_list(state, legal_actions)


class QLearningWithApproximation(LearningAlgorithm):
    def __init__(self, actions=None, features=None, learning_rate=1,
        discount_factor=1, exploration_rate=0):
        super(QLearningWithApproximation, self).__init__()
        self.actions = actions
        self.features = features
        self.learning_rate = learning_rate
        self.discount_factor = discount_factor
        self.previous_state = None
        self.exploration_rate = exploration_rate

        self.weights = {}
        self._init_weights()

    def _init_weights(self):
        for action in self.actions:
            self.weights[str(action)] = [random.random() for _ in range(len(self.features))]

    def get_weights(self):
        return self.weights

    def set_weights(self, weights):
        self.weights = weights

    def get_q_value(self, state, action):
        q_value = 0

        for weight, feature in zip(self.weights[str(action)], self.features):
            q_value += weight*feature(state, action)

        return q_value

    def _get_max_action_from_list(self, state, action_list):
        """Get the action with maximum estimated value from the given list of
        actions.

        state -- Environment state.
        action_list -- Actions to be evaluated.
        """
        actions = filter(lambda a: a in action_list, self.actions)
        values = [self.get_q_value(state, action) for action in actions]
        max_value = max(values)
        max_actions = [action
            for action in actions if self.get_q_value(state, action) == max_value]

        return random.choice(max_actions)

    def get_max_action(self, state):
        return self._get_max_action_from_list(state, self.actions)

    def get_max_q_value(self, state):
        action = self.get_max_action(state)
        return self.get_q_value(state, action)

    def _update_weights(self, action, delta):
        self.weights[str(action)] = [weight + self.learning_rate*delta*feature(self.previous_state, action) for weight, feature in zip(self.weights[str(action)], self.features)]


    def learn(self, state, action, reward):
        if self.previous_state:
            delta = (reward + self.discount_factor*self.get_max_q_value(state)
                - self.get_q_value(self.previous_state, action))
            self._update_weights(action, delta)

        self.previous_state = copy.deepcopy(state)

    def _explore(self):
        return random.choice(self.actions)

    def _exploit(self, state):
        return self._get_max_action_from_list(state, self.actions)

    def act(self, state):
        p = random.random()

        if p < self.exploration_rate:
            return self._explore()
        else:
            return self._exploit(state)
