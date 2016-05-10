class Agent(object):
    def __init__(self, action_space, state_space):
        self.action_space = action_space
        self.state_space = state_space

    def learn(self, state, reward):
        raise NotImplementedError

    def act(self, state):
        raise NotImplementedError


class Environment(object):
    def reset(self):
        raise NotImplementedError

    def step(self, action):
        raise NotImplementedError

    def is_finished(self):
        raise NotImplementedError

    def render(self):
        raise NotImplementedError



class Space(object):
    def sample(self, seed=0):
        raise NotImplementedError


import random

import numpy as np


class DiscreteSpace(Space):
    def __init__(self, n):
        self.n = n

    def sample(self):
        return np.random.randint(self.n)


class ContinuousSpace(Space):
    def __init__(self, min_, max_):
        self.min = min_
        self.max = max_

    def sample(self):
        return (self.max - self.min) * np.random.random_sample() + self.min


class RandomAgent(Agent):
    def learn(self, state, reward):
        pass

    def act(self, state):
        return self.action_space.sample()


class FixedAgent(Agent):
    def learn(self, state, reward):
        pass

    def act(self, state):
        return self.action_space[0]


class QLearningAgent(Agent):
    def __init__(self, action_space, state_space):
        super(QLearningAgent, self).__init__(action_space, state_space)
        self.q = {}
        self.previous_action = None
        self.previous_state = None
        self.learning_rate = 0.1
        self.discount_factor = 0.9
        self.exploration_rate = 0.1
        self.can_explore = True

    def get_q_value(self, state, action):
        if state not in self.q:
            self.q[state] = {action: 0 for action in range(self.action_space.n)}

        return self.q[state][action]

    def get_max_q_value(self, state):
        if state not in self.q:
            self.q[state] = {action: 0 for action in range(self.action_space.n)}

        return max(self.q[state].values())

    def learn(self, state, reward):
        previous_q_value = self.get_q_value(self.previous_state, self.previous_action)
        expected_q_value = self.get_max_q_value(state)
        updated_q_value = previous_q_value + self.learning_rate*(reward + self.discount_factor*expected_q_value - previous_q_value)
        self.q[self.previous_state][self.previous_action] = updated_q_value

    def act(self, state):
        if self.can_explore and random.random() < self.exploration_rate:
            action = self.explore()
        else:
            action = self.exploit(state)

        self.previous_state = state
        self.previous_action = action
        return action

    def explore(self):
        return self.action_space.sample()

    def exploit(self, state):
        if state in self.q:
            max_value = self.get_max_q_value(state)
            best_actions = [action for action, value in self.q[state].items() if value == max_value]
            action = random.choice(best_actions)
        else:
            action = self.action_space.sample()

        return action

class WindyEnvironment(Environment):
    def __init__(self):
        super(WindyEnvironment, self).__init__()
        self.initial_coordinates = [1, 0]
        self.actions = [[0, 1], [-1, 0], [0, -1], [1, 0], [0, 0]]
        self.action_space = DiscreteSpace(len(self.actions))
        self.rows = 7
        self.cols = 10
        self.state_space = DiscreteSpace(self.rows * self.cols)
        self.goal_coordinates = [1, 7]
        self.water_coordinates = [[0, 3], [0, 4], [2, 3], [2, 4], [3, 3], [3, 4]]
        self.wind_frequency = 0.75
        self.num_actions = len(self.actions)
        self.num_states = self.rows*self.cols

    def get_actions(self):
        return self.actions

    def get_reward(self):
        if self.agent_coordinates in self.water_coordinates:
          reward = -100
        elif self.agent_coordinates == self.goal_coordinates:
          reward = 100
        else:
          reward = -1

        return reward

    def get_state(self):
        return self.agent_coordinates

    def index_to_action(self, index):
        return self.actions[index]

    def state_to_index(self, state):
        return state[1]*self.rows + state[0]

    def reset(self):
        self.agent_coordinates = self.initial_coordinates
        state = self.state_to_index(self.agent_coordinates)
        return state

    def step(self, action):
        # wind
        if np.random.random() < self.wind_frequency:
          wind_direction = np.random.randint(self.num_actions)
          wind_action = [self.actions[wind_direction][0], self.actions[wind_direction][1]]
        else:
          wind_action = [0, 0]

        # update coordinates
        action_coordinates = self.index_to_action(action)
        self.agent_coordinates = [
            min(max(self.agent_coordinates[0] + action_coordinates[0] + wind_action[0], 0), self.rows - 1),
            min(max(self.agent_coordinates[1] + action_coordinates[1] + wind_action[1], 0), self.cols - 1),
        ]

        state = self.state_to_index(self.agent_coordinates)
        reward = self.get_reward()
        return state, reward

    def is_finished(self):
        return self.agent_coordinates == self.goal_coordinates

    def render(self):
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


def main():
    import time

    environment = WindyEnvironment()
    agent = QLearningAgent(environment.action_space, environment.state_space)

    num_episodes = 1000
    max_steps = 10000

    # Learning
    for _ in range(num_episodes):
        state = environment.reset()

        for _ in range(max_steps):
            action = agent.act(state)
            state, reward = environment.step(action)
            agent.learn(state, reward)

            if environment.is_finished():
                break

    # Acting
    agent.can_explore = False
    state = environment.reset()

    for _ in range(max_steps):
        action = agent.act(state)
        print agent.q[state]
        print agent.q[state][action]
        state, reward = environment.step(action)
        environment.render()
        print reward

        if environment.is_finished():
            break

        time.sleep(0.5)


if __name__ == '__main__':
    main()
