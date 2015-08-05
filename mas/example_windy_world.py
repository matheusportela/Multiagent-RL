#!/usr/bin/env python

import random
import pymas


class QLearner(object):
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


class RandomWorldAgent(pymas.Agent):
    agent_id = None

    def __init__(self):
        super(RandomWorldAgent, self).__init__()
        self.x = 0
        self.y = 0
        self.rows = 7
        self.cols = 10
        self.learner = QLearner(initial_state=0, num_states=self.rows*self.cols,
            num_actions=4, learning_rate=1, discount_factor=1)
        self.last_action = 0

        RandomWorldAgent.agent_id = self.id

    def __str__(self):
        return str(self.id)

    def on_run(self):
        state = self.coordinates_to_state((self.x, self.y))
        action = self.learner.act(state)
        self.last_action = action
        self.send_message(pymas.Message(receiver=WindyWaterAgent.simulator_id, data=action))

    def on_receive_message(self, message):
        if (message.sender == WindyWaterAgent.simulator_id):
            if message.data == 'stop':
                self.stop()
            else:
                self.x, self.y = message.data['coordinates']
                state = self.coordinates_to_state((self.x, self.y))
                self.learner.learn(state, self.last_action, message.data['reward'])

    def coordinates_to_state(self, coordinates):
        return coordinates[0]*self.cols + coordinates[1]


class WindyWaterAgent(pymas.Agent):
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

    simulator_id = None

    def __init__(self, wind_frequency=0.1):
        super(WindyWaterAgent, self).__init__()
        self.initial_coordinates = [1, 0]
        self.actions = [[0, 1], [-1, 0], [0, -1], [1, 0]]
        self.rows = 7
        self.cols = 10
        self.goal_coordinates = [1, 7]
        self.water_coordinates = [[0, 3], [0, 4], [2, 3], [2, 4], [3, 3], [3, 4]]
        self.wind_frequency = wind_frequency
        self.num_actions = len(self.actions)
        self.num_states = self.rows*self.cols
        self.num_episodes = 100
        self.current_episode = 0

        WindyWaterAgent.simulator_id = self.id

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
        return self.agent_coordinates

    def calculate_reward(self):
        if (self.agent_coordinates in self.water_coordinates):
          reward = -100
        elif (self.agent_coordinates == self.goal_coordinates):
          reward = 100
        else:
          reward = -1

        return reward

    def is_episode_finished(self):
        return (self.agent_coordinates == self.goal_coordinates)

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

    def on_start(self):
        self.prepate_new_episode()

    def on_receive_message(self, message):
        coordinates = self.calculate_state(message.data)
        reward = self.calculate_reward()
        message = pymas.Message(receiver=RandomWorldAgent.agent_id)
        message.data = {
            'coordinates': coordinates,
            'reward': reward,
        }
        self.send_message(message)

    def on_run(self):
        self.print_map()

        if self.is_episode_finished():
            self.current_episode += 1

            if self.current_episode == self.num_episodes:
                self.send_message(pymas.Message(receiver=RandomWorldAgent.agent_id, data='stop'))
                self.stop()
            else:
                self.prepate_new_episode()

    def on_stop(self):
        print 'Finish simulation'


if __name__ == '__main__':
    system = pymas.System()
    system.add_agent(WindyWaterAgent)
    system.add_agent(RandomWorldAgent)
    system.run(sleep=0)