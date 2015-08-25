"""Collection of reinforcement learning algorithms"""

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


class QLearning(object):
    """Q-learning algorithm implementation.

    Q-learning is a model free reinforcement learning algorithm that tries and
    learning state values and chooses actions that maximize the expected
    discounted reward for the current state.

    Instance variables:
    current_state -- State in which the algorithm currently is.
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
        self.current_state = initial_state
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

    def update_state(self, new_state):
        """Update Q Learning current state.

        Parameters:
        new_state -- State to which the learning algorithm is going.
        """
        self.current_state = new_state

    def initialize_unknown_state(self, state):
        """Initialize Q-values for states that were not previously seen.

        Parameters:
        state -- Environment state.
        """
        if state not in self.q_values:
            self.q_values[state] = {}
            for action_ in self.actions:
                self.q_values[state][action_] = 0

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

    def get_max_action(self, state):
        """Get the action with maximum estimated value.

        Parameters:
        state -- Environment state.
        """
        self.initialize_unknown_state(state)

        actions = [action for action in self.q_values[state]]
        values = [self.q_values[state][action]
            for action in self.q_values[state]]
        max_value = max(values)
        max_actions = [action
            for action in actions if self.q_values[state][action] == max_value]

        return random.choice(max_actions)

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
        old_value = self.get_q_value(self.current_state, action)
        next_expected_value = self.get_max_q_value(state)
        new_value = (old_value + self.learning_rate*(reward + self.discount_factor*next_expected_value - old_value))
        self.set_q_value(self.current_state, action, new_value)
        self.update_state(state)

    def act(self, state, legal_actions):
        """Select the best legal action for the given state.

        Parameters:
        state -- Agent state to select an action.
        """
        actions = filter(lambda a: a in legal_actions, self.q_values[state])
        values = [self.q_values[state][action] for action in actions]
        max_value = max(values)
        max_actions = [action
            for action in actions if self.q_values[state][action] == max_value]

        return random.choice(max_actions)