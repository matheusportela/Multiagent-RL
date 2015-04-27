#!/usr/bin/python


class Learner(object):
    """Learning abstract class.

    Every learning algorithm must inherit from this class and implement it's
    methods correctly.
    """

    def learn(self, state, action, reward):
        """Basic learning method interface.

        This method is the interface for learning. Given a pair (state, action),
        the learning algorithm must update it's internals to accomodate the new
        information.

        Parameters:
        state -- State where the state went after executing the action.
        action -- Action executed.
        reward -- Reward received after executing the action.
        """
        raise NotImplementedError

    def act(self, state):
        """Basic action selection method interface.

        This method is the interface for action selection. Given the system
        state, the learning algorith must select one selection to be executed by
        the agent following it's own rationality.

        Parameters:
        state -- Current system state.

        Return:
        Action for the current selected by the learning algorith.
        """

        raise NotImplementedError


class QLearner(Learner):
    """Q-learning algorithm implementation.

    Q-learning is a model free reinforcement learning algorithm that tries and
    learning state values and chooses actions that maximize the expected
    discounted reward for the current state.

    Instance variables:
    current_state -- State in which the algorithm currently is.
    q_values -- Matrix that stores the value for a (state, action) pair.
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

    def __init__(self, initial_state=0, num_states=0, num_actions=0,
        learning_rate=1, discount_factor=1):
        """Constructor.

        Parameters:
        initial_state -- State where the algorithm begins.
        num_states -- Number of states to be represented.
        num_actions -- Number of actions to be represented.
        """
        self.current_state = initial_state
        self.q_values = QValues(num_states=num_states, num_actions=num_actions)
        self.learning_rate = learning_rate
        self.discount_factor = discount_factor

    def update_state(self, new_state):
        """Update Q Learning current state.

        Parameters:
        new_state -- State to which the learning algorithm is going.
        """
        self.current_state = new_state

    def learn(self, state, action, reward):
        """Learn by updating the (state, action) reward.

        Learn by applying the reward received when transitioning from the
        current state to the new one by executing an action.

        Parameters:
        state -- Agent state after executing the action.
        action -- Executed action.
        reward -- Reward received after executing the action.
        """
        old_value = self.q_values.get(self.current_state, action)
        next_expected_value = self.q_values.get_max_value(state)
        new_value = old_value + self.learning_rate*(reward + self.discount_factor*next_expected_value - old_value)
        self.q_values.set(self.current_state, action, new_value)

        self.update_state(state)

    def act(self, state):
        """Select an action for the given state.

        Parameters:
        state -- Agent state to select an action.
        """
        return self.q_values.get_max_action(state)

class QValues(object):
    """Container for Q values.

    Instance variables:
    num_states -- Number of states that will be stored.
    num_actions -- Number of actions that will be stored.
    q_values -- Container for Q values.
    """

    def __init__(self, num_states=0, num_actions=0):
        self.num_states = num_states
        self.num_actions = num_actions
        self.q_values = [[0 for _ in xrange(num_actions)] for _ in xrange(num_states)]

    def get(self, state, action):
        """Get stored Q value for a (state, action) pair.

        Parameters:
        state -- State index.
        action -- Action index.
        """
        return self.q_values[state][action]

    def set(self, state, action, q_value):
        """Set Q value for a (state, action) pair.

        Parameters:
        state -- State index.
        action -- Action index.
        q_value -- Q value to be stored.
        """
        self.q_values[state][action] = q_value

    def get_max_value(self, state):
        """Returns the maximum Q value possible for the given state.

        Parameters:
        state -- State from which to find the maximum Q value possible.
        """
        return max(self.q_values[state])

    def get_max_action(self, state):
        """Returns the action index for which the Q value is maximum for the
        given state.

        Parameters:
        state -- State from which to find the action.
        """
        max_value = self.get_max_value(state)
        return self.q_values[state].index(max_value)