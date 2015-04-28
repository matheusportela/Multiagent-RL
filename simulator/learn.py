#!/usr/bin/python

import random

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

    def __str__(self):
        return ('Q-learning\n' + str(self.q_values))


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
        actions = [action for action, value in enumerate(self.q_values[state]) if value == max_value]
        return random.choice(actions)

    def __str__(self):
        output = []

        for state, actions in enumerate(self.q_values):
            for action, value in enumerate(actions):
                if value != 0.0:
                    output.append('(%d, %d): %d' % (state, action, value))

        return '\n'.join(output)


class SystemAdapter(object):
    """Connects a computational agent to it's simulation or physical system."""

    def run(self, measurements):
        """System basic sense-think-act cycle.

        This method acts as the interface to receive measurements from the
        system, learn from it and select a new action the be executed.

        A measurement includes all necessary information for the system, such as
        rewards (for a reinforcement learning algorithm), agents positions,
        environment information etc.
        """
        raise NotImplementedError


class PacmanMeasurements(object):
    """Container for measurements in UC Berkeley Pacman game.

    Instance variables:
    pacman_position -- Current Pacman position.
    ghosts_positions -- Current position for each ghost.
    """

    def __init__(self, pacman_position=None, ghosts_positions=None, action=None,
            reward=None):
        self.pacman_position = pacman_position
        self.ghosts_positions = ghosts_positions
        self.action = action
        self.reward = reward


    def __str__(self):
        output = []
        output.append('Measurements:')
        output.append('Pacman position: %s' % str(self.pacman_position))
        output.extend(['Ghost %d position: %s' % (i, pos)
            for i, pos in enumerate(self.ghosts_positions)])
        output.append('Action: %s' % self.action)
        output.append('Reward: %d' % self.reward)
        return '\n'.join(output)


class PacmanActions(object):
    """Container for actions in UC Berkeley Pacman game.

    Instance variables:
    pacman_action -- Action to be executed by Pacman.
    ghosts_actions -- Actions to be executed by each ghost.
    """

    def __init__(self, pacman_action=None, ghosts_actions=None):
        self.pacman_action = pacman_action
        self.ghosts_actions = ghosts_actions


class PacmanSystemAdapter(SystemAdapter):
    """System adapter for UC Berkeley Pacman game."""

    def __init__(self, width, height):
        self.width = width
        self.height = height
        num_states = width*height
        num_actions = 5
        self.learn_algorithm = QLearner(num_states=num_states,
            num_actions=num_actions)
        self.action_to_index = {
            'Stop': 0,
            'North': 1,
            'South': 2,
            'East': 3,
            'West': 4,
        }
        self.index_to_action = dict(zip(self.action_to_index.values(), self.action_to_index.keys()))

    def convert_position_to_index(self, position):
        return (position[0]*self.width + position[1])

    def run(self, measurements):
        state_index = self.convert_position_to_index(measurements.pacman_position)
        action_index = self.action_to_index[measurements.action]
        self.learn_algorithm.learn(state_index, action_index, measurements.reward)
        print str(self.learn_algorithm)
        new_action_index = self.learn_algorithm.act(state_index)
        new_action = self.index_to_action[new_action_index]
        return new_action