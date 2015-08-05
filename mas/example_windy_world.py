#!/usr/bin/env python

import random
import pymas


class RandomWorldAgent(pymas.Agent):
    agent_id = None

    def __init__(self):
        super(RandomWorldAgent, self).__init__()
        self.x = 0
        self.y = 0

        RandomWorldAgent.agent_id = self.id

    def __str__(self):
        return str(self.id)

    def on_run(self):
        action = random.randrange(4)
        self.send_message(pymas.Message(receiver=WindyWaterAgent.simulator_id, data=action))

    def on_receive_message(self, message):
        if (message.sender == WindyWaterAgent.simulator_id):
            if message.data == 'stop':
                self.stop()
            else:
                print message.data

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
        self.num_episodes = 10
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