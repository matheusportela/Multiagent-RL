#!/usr/bin/env python

from __future__ import division
from mas import pymas
import random
import time
import learning

class Cell(object):
    def __init__(self):
        self.objects = []

    def __str__(self):
        if len(self.objects) == 0:
            return '.'
        return str(self.objects[-1])

    def add(self, object):
        self.objects.append(object)

    def remove(self, object):
        self.objects = [o for o in self.objects if id(o) != id(object)]


class EmptyCell(Cell):
    def __init__(self):
        super(EmptyCell, self).__init__()


class Map(object):
    empty = None

    def __init__(self, width=1, height=1):
        self.width = width
        self.height = height
        self.clear()

    def __str__(self):
        string = []

        for cell_line in self.cells:
            for cell in cell_line:
                string.append(str(cell))
                string.append(' ')
            string.append('\n')

        return ''.join([str(line) for line in string])

    def add_object(self, object, x, y):
        self.cells[x][y].add(object)

    def clear(self):
        self.cells = [[EmptyCell() for _ in range(self.width)] for _ in
                       range(self.height)]


class EGreedyExplorer(object):
    """e-greedy exploration algorithm.

    Selects the suggested action or another random action with the given
    exploration_frequency.
    """
    def __init__(self, actions=[], exploration_frequency=0.0):
        self.actions = actions
        self.exploration_frequency = exploration_frequency

    def select_action(self, suggested_action):
        if random.random() < self.exploration_frequency:
            return random.choice(self.actions)
        else:
            return suggested_action


class LearningAgent(pymas.Agent):
    def __init__(self):
        super(LearningAgent, self).__init__()
        self.state = (0, 0, 0, 0)
        self.actions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        self.learner = learning.QLearning(initial_state=self.state,
            learning_rate=0.9, discount_factor=0.9, actions=self.actions)
        self.explorer = EGreedyExplorer(actions=self.actions,
            exploration_frequency=0.1)
        self.last_action = self.actions[0]

    def on_run(self):
        suggested_action = self.learner.act(self.state)
        action = self.explorer.select_action(suggested_action)
        self.last_action = action
        self.send_message(pymas.Message(receiver=SimulatorAgent.simulator_id, data=action))

    def on_receive_message(self, message):
        if (message.sender == SimulatorAgent.simulator_id):
            if message.data == 'stop':
                self.stop()
            else:
                self.state = message.data['state']
                reward = message.data['reward']
                self.learner.learn(self.state, self.last_action, reward)


class PredatorAgent(LearningAgent):
    def __init__(self):
        super(PredatorAgent, self).__init__()

    def __str__(self):
        return 'P'

    def on_stop(self):
        print 'Predator learned Q-values'
        print str(self.learner)


class PreyAgent(LearningAgent):
    def __init__(self):
        super(PreyAgent, self).__init__()

    def __str__(self):
        return 'p'

    def on_stop(self):
        print 'Prey learned Q-values'
        print str(self.learner)


class SimulatorAgent(pymas.Agent):
    simulator_id = 0
    map_rows = 7
    map_cols = 7
    num_episodes = 2000
    sleep_time = 0.25

    def __init__(self):
        super(SimulatorAgent, self).__init__()
        self.map = Map(SimulatorAgent.map_rows, SimulatorAgent.map_cols)
        self.agents = []
        self.prey_agent = None
        self.predator_agents = []
        self.current_episode = 0
        SimulatorAgent.simulator_id = self.id

    def register_agent(self, agent):
        agent.x = random.randrange(self.map.width)
        agent.y = random.randrange(self.map.height)
        self.agents.append(agent)

        if type(agent) == PreyAgent:
            self.prey_agent = agent
        elif type(agent) == PredatorAgent:
            self.predator_agents.append(agent)

    def prepate_new_episode(self):
        for agent in self.agents:
            agent.x = random.randrange(0, SimulatorAgent.map_rows)
            agent.y = random.randrange(0, SimulatorAgent.map_cols)

    def on_start(self):
        self.prepate_new_episode()

    def on_run(self):
        self.map.clear()
        for agent in self.agents:
            self.map.add_object(agent, agent.x, agent.y)

        if self.current_episode == SimulatorAgent.num_episodes - 1:
            for predator in self.predator_agents:
                predator.explorer.exploration_frequency = 0
            print str(self.map)
            time.sleep(SimulatorAgent.sleep_time)

        for predator in self.predator_agents:
            state = (predator.x, predator.y, self.prey_agent.x, self.prey_agent.y)

            if (predator.x, predator.y) == (self.prey_agent.x, self.prey_agent.y):
                self.send_message(pymas.Message(receiver=predator.id,
                    data={
                        'state': state,
                        'reward': 100,
                    }))
                self.send_message(pymas.Message(receiver=self.prey_agent.id,
                    data={
                        'state': state,
                        'reward': -100,
                    }))
            else:
                self.send_message(pymas.Message(receiver=predator.id,
                    data={
                        'state': state,
                        'reward': -1,
                    }))
                self.send_message(pymas.Message(receiver=self.prey_agent.id,
                    data={
                        'state': state,
                        'reward': 1,
                    }))

        for predator in self.predator_agents:
            if (predator.x, predator.y) == (self.prey_agent.x, self.prey_agent.y):
                print 'Predator %d capturated prey at %d, %d' % (predator.id,
                    predator.x, predator.y)

                self.current_episode += 1

                if self.current_episode == SimulatorAgent.num_episodes:
                    self.finish_simulation()
                else:
                    self.prepate_new_episode()

    def finish_simulation(self):
        for agent in self.agents:
            self.send_message(pymas.Message(receiver=agent.id, data='stop'))

        self.stop()

    def on_receive_message(self, message):
        action = message.data
        agent = None

        for a in self.agents:
            if a.id == message.sender:
                agent = a

        if not agent:
            return

        agent.x += action[0]
        agent.y += action[1]

        if agent.x >= self.map.width:
            agent.x = self.map.width - 1
        elif agent.x < 0:
            agent.x = 0

        if agent.y >= self.map.height:
            agent.y = self.map.height - 1
        elif agent.y < 0:
            agent.y = 0


if __name__ == '__main__':
    system = pymas.System()
    simulator = system.add_agent(SimulatorAgent)
    simulator.register_agent(system.add_agent(PreyAgent))
    simulator.register_agent(system.add_agent(PredatorAgent))
    system.run(sleep=0)