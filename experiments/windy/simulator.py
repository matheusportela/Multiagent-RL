import random
import time


class WindyWorldSimulator(object):
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

    S: Initial state
    W: Water that gives penalty
    G: Goal state
    *: Free cell

    Each step gives a reward of -1, going into the water rewards -100 and
    reaching the goal state rewards 100.
    """

    def __init__(self, wind_frequency=0.1, sleep=0.1):
        super(WindyWorldSimulator, self).__init__()
        self.initial_coordinates = [1, 0]
        self.agent_coordinates = self.initial_coordinates
        self.actions = [[0, 1], [-1, 0], [0, -1], [1, 0]]
        self.rows = 7
        self.cols = 10
        self.goal_coordinates = [1, 7]
        self.water_coordinates = [
            [0, 3], [0, 4], [2, 3], [2, 4], [3, 3], [3, 4]
        ]
        self.wind_frequency = wind_frequency
        self.num_actions = len(self.actions)
        self.num_states = self.rows*self.cols
        self.sleep = sleep
        self.score = 0

    def __repr__(self):
        lines = [[]]

        for i in xrange(self.rows):
            for j in xrange(self.cols):
                if [i, j] == self.agent_coordinates:
                    lines[-1].append('A')
                elif [i, j] == self.initial_coordinates:
                    lines[-1].append('S')
                elif [i, j] == self.goal_coordinates:
                    lines[-1].append('G')
                elif [i, j] in self.water_coordinates:
                    lines[-1].append('W')
                else:
                    lines[-1].append('*')
            lines.append([])

        return '\n'.join(' '.join(line) for line in lines)

    def prepate_new_episode(self):
        self.agent_coordinates = self.initial_coordinates
        self.score = 0

    def get_state(self):
        return (self.agent_coordinates[0]*self.rows +
                self.agent_coordinates[1]*self.cols)

    def generate_state(self, action):
        # wind
        if random.random() < self.wind_frequency:
            wind_direction = random.randrange(0, self.num_actions)
            wind_action = [self.actions[wind_direction][0],
                           self.actions[wind_direction][1]]
        else:
            wind_action = [0, 0]

        # state generation
        self.agent_coordinates = [
            min(max(self.agent_coordinates[0] +
                    self.actions[action][0] + wind_action[0], 0),
                self.rows - 1),
            min(max(self.agent_coordinates[1] +
                    self.actions[action][1] + wind_action[1], 0),
                self.cols - 1),
        ]
        return self.agent_coordinates

    def get_reward(self):
        if (self.agent_coordinates in self.water_coordinates):
            reward = -100
        elif (self.agent_coordinates == self.goal_coordinates):
            reward = 100
        else:
            reward = -1

        return reward

    def is_finished(self):
        return self.agent_coordinates == self.goal_coordinates

    def start(self):
        self.prepate_new_episode()

    def step(self, action):
        coordinates = self.generate_state(action)
        self.score += self.get_reward()
        time.sleep(self.sleep)
        return self.get_state()


if __name__ == '__main__':
    simulator = WindyWorldSimulator()
    print simulator
