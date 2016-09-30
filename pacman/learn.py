#!/usr/bin/python

import random


class ProblemController(object):
    """Controls the execution of episodes in a given problem adapter and with an
    agent.
    """
    def __init__(self, num_episodes, problem_adapter, agent):
        self.num_episodes = num_episodes
        self.problem_adapter = problem_adapter
        self.agent = agent

    def run(self):
        avg_reward, avg_steps = self.execute_episodes()
        print 'Average reward:', avg_reward
        print 'Average steps:', avg_steps

    def execute_episodes(self):
        episodes_rewards = []
        episodes_steps = []

        for _ in range(self.num_episodes):
            cumulative_reward, steps = self.execute_episode(self.problem_adapter, self.agent)
            episodes_rewards.append(cumulative_reward)
            episodes_steps.append(steps)

        avg_reward = sum(episodes_rewards)/self.num_episodes
        avg_steps = sum(episodes_steps)/self.num_episodes

        return avg_reward, avg_steps

    def execute_episode(self, problem_adapter, agent):
        cumulative_reward = 0
        steps = 0
        state = self.problem_adapter.initial_state
        self.problem_adapter.prepate_new_episode()

        while not self.problem_adapter.is_episode_finished():
            action = agent.act(state)
            state = self.problem_adapter.calculate_state(action)
            reward = self.problem_adapter.calculate_reward(state)
            agent.learn(action, state, reward)

            cumulative_reward += reward
            steps += 1

        return cumulative_reward, steps



class Agent(object):
    """Agent capable of learning and exploring.

    All action, state and reward variables must be numerical values. Besides,
    action and state must be an uniquely identified integer.
    """
    def __init__(self):
        self.learning_element = None
        self.exploration_element = None

    def learn(self, action, state, reward):
        """Executes the learning algorithm."""
        self.learning_element.learn(state, action, reward)

    def act(self, state):
        """Selects an action to be executed by consulting both the learning and
        exploration algorithms.
        """
        suggested_action = self.learning_element.act(state)
        selected_action = self.exploration_element.select_action(suggested_action)
        return selected_action


class ProblemAdapter(object):
    """Adapter for a specific learning problem.

    Problem adapter stores specific information about the problem where the
    agent is running.
    """
    def __init__(self, initial_state=0, num_actions=1, num_states=1):
        self.initial_state = initial_state
        self.num_actions = num_actions
        self.num_states = num_states

    def prepare_new_episode(self):
        """Preparations for a new episode to be executed."""
        raise NotImplementedError

    def calculate_state(self, action):
        """Calculate the new agent state for a given action."""
        raise NotImplementedError

    def calculate_reward(self, state):
        """Calculate the reward for the given state."""
        raise NotImplementedError

    def is_episode_finished(self):
        """Checks whether the current episode has finished."""
        raise NotImplementedError


class Learner(object):
    """Learning algorithm interface.

    A learning algorithm can select the best-suited action for any state and
    adapt it's belief according to reward information.
    """
    def learn(self, state, action, reward):
        """Learn state-action value by incorporating the reward information."""
        raise NotImplementedError

    def act(self, state):
        """Select an action for the given state."""
        raise NotImplementedError


class Explorer(object):
    """Exploration algorithm interface.

    An exploration algorithm is used by an agent to select actions other than
    the optimal one, potentially increasing the rewards in the long run.
    """
    def select_action(self, suggested_action):
        """Select an action given the one suggested by the learning algorithm."""
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
        output = ['\t%d' % action for action in range(self.num_actions)]
        output.append('\n')
        for state, values in enumerate(self.q_values):
            output.append('%d' % state)
            for value in values:
                output.append('\t%1.1f' % value)
            output.append('\n')
        return ''.join(output)


class EGreedyExplorer(Explorer):
    """e-greedy exploration algorithm.

    Selects the suggested action or another random action with the given
    exploration_frequency.
    """
    def __init__(self, num_actions=1, exploration_frequency=0.0):
        self.actions = range(num_actions)
        self.exploration_frequency = exploration_frequency

    def select_action(self, suggested_action):
        if random.random() < self.exploration_frequency:
            return random.choice(self.actions)
        else:
            return suggested_action


class QAgent(Agent):
    """Example agent with Q-learning and e-greedy exploration algorithms."""
    def __init__(self, initial_state, num_states, num_actions):
        self.learning_element = QLearner(
            initial_state=initial_state,
            num_states=num_states,
            num_actions=num_actions,
            learning_rate=0.9,
            discount_factor=0.9,
        )
        self.exploration_element = EGreedyExplorer(
            num_actions=num_actions,
            exploration_frequency=0.1,
        )


class WindyWaterAdapter(ProblemAdapter):
    """Windy water example problem.

    The agent lives in the following world:
    * * * W W * * * * *
    S * * * * * * G * *
    * * * W W * * * * *
    * * * W W * * * * *
    * * * * * * * * * *
    * * * * * * * * * *
    * * * * * * * * * *

    where:
    S: initial state
    W: water that gives penalty
    G: goal state

    Each step gives a reward of -1, going into the water rewards -100 and
    reaching the goal state rewards 100.
    """
    def __init__(self, wind_frequency=0):
        self.initial_coordinates = [1, 0]
        self.actions = [[0, 1], [-1, 0], [0, -1], [1, 0]]
        self.rows = 7
        self.cols = 10
        self.goal_coordinates = [1, 7]
        self.water_coordinates = [[0, 3], [0, 4], [2, 3], [2, 4], [3, 3], [3, 4]]
        self.wind_frequency = wind_frequency

        super(WindyWaterAdapter, self).__init__(
            initial_state=self.coordinates_to_state(self.initial_coordinates),
            num_actions=len(self.actions),
            num_states=self.rows*self.cols,
        )

    def prepate_new_episode(self):
        self.agent_coordinates = self.initial_coordinates

    def calculate_state(self, action):
        # wind
        if random.random() < self.wind_frequency:
          wind_direction = random.randrange(0, self.num_actions)
          wind_action = [self.actions[wind_direction][0], self.actions[wind_direction][1]]
        else:
          wind_action = [0, 0]

        # state generation
        self.agent_coordinates = [
            min(max(self.agent_coordinates[0] + self.actions[action][0] + wind_action[0], 0), self.rows - 1),
            min(max(self.agent_coordinates[1] + self.actions[action][1] + wind_action[1], 0), self.cols - 1),
        ]
        state = self.coordinates_to_state(self.agent_coordinates)

        return state

    def calculate_reward(self, state):
        if (self.agent_coordinates in self.water_coordinates):
          reward = -100
        elif (self.agent_coordinates == self.goal_coordinates):
          reward = 100
        else:
          reward = -1

        return reward

    def is_episode_finished(self):
        return (self.agent_coordinates == self.goal_coordinates)

    def coordinates_to_state(self, coordinates):
        return coordinates[0]*self.cols + coordinates[1]

    def print_map(self):
        print
        for i in xrange(self.rows):
            for j in xrange(self.cols):
                if [i,j] == self.agent_coordinates:
                    print "A",
                elif [i,j] == self.initial_coordinates:
                    print "S",
                elif [i,j] == self.goal_coordinates:
                    print "G",
                elif [i,j] in self.water_coordinates:
                    print "W",
                else:
                    print "*",
            print