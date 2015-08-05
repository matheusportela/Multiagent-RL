#!/usr/bin/env python

from __future__ import division
import pymas
import random


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


class RandomWorldAgent(pymas.Agent):
    def __init__(self):
        super(RandomWorldAgent, self).__init__()
        self.x = 0
        self.y = 0

    def __str__(self):
        return str(self.id)

    def on_run(self):
        action = random.randrange(4)
        self.send_message(pymas.Message(receiver=SimulatorAgent.simulator_id, data=action))

    def on_receive_message(self, message):
        if (message.sender == SimulatorAgent.simulator_id
            and message.data == 'stop'):
            self.stop()


class SimulatorAgent(pymas.Agent):
    simulator_id = 0

    def __init__(self):
        super(SimulatorAgent, self).__init__()
        self.map = Map(7, 7)
        self.agents = []
        self.goal = (1, 1)
        SimulatorAgent.simulator_id = self.id

    def register_agent(self, agent):
        agent.x = random.randrange(self.map.width)
        agent.y = random.randrange(self.map.height)
        self.agents.append(agent)

    def on_run(self):
        self.map.clear()
        for agent in self.agents:
            self.map.add_object(agent, agent.x, agent.y)

        print str(self.map)

        for agent in self.agents:
            if (agent.x, agent.y) == self.goal:
                print 'Agent %d reached goal at %d, %d' % (agent.id,
                    self.goal[0], self.goal[1])
                self.finish_simulation()

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

        if action == 0:
            agent.x += 1
        elif action == 1:
            agent.x -= 1
        elif action == 2:
            agent.y += 1
        elif action == 3:
            agent.y -= 1

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
    simulator.register_agent(system.add_agent(RandomWorldAgent))
    simulator.register_agent(system.add_agent(RandomWorldAgent))
    simulator.register_agent(system.add_agent(RandomWorldAgent))
    simulator.register_agent(system.add_agent(RandomWorldAgent))
    system.run(sleep=0.1)