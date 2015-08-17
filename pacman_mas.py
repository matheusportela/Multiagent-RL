#!/usr/bin/env python

from __future__ import division
from mas import pymas
import communication as comm
import pickle
import random


class PacmanAgent(pymas.Agent):
    def __init__(self):
        super(PacmanAgent, self).__init__()
        self.simulation_agent = None
        self.actions = ['North', 'South', 'East', 'West', 'Stop']

    def register_simulation_agent(self, agent):
        self.simulation_agent = agent

    def choose_action(self, position):
        return random.choice(self.actions)

    def send_action(self, action):
        msg = pymas.Message(receiver=self.simulation_agent.id, data=action)
        self.send_message(msg)

    def on_receive_message(self, message):
        print 'Received message: {}'.format(message)
        position = message.data
        action = self.choose_action(position)
        self.send_action(action)


class SimulatorRoutingAgent(pymas.Agent):
    def __init__(self):
        super(SimulatorRoutingAgent, self).__init__()
        self.pacman_agent = None
        self.server = comm.Server()

    def register_pacman_agent(self, agent):
        self.pacman_agent = agent

    def on_run(self):
        pacman_position = pickle.loads(self.server.recv())
        self.send_pacman_position(pacman_position)

    def send_pacman_position(self, pacman_position):
        msg = pymas.Message(receiver=self.pacman_agent.id, data=pacman_position)
        self.send_message(msg)

    def on_receive_message(self, message):
        print 'Received reply: {}'.format(message)
        pacman_action = message.data
        self.server.send(pacman_action)


if __name__ == '__main__':
    system = pymas.System()
    pacman_agent = system.add_agent(PacmanAgent)
    simulator_agent = system.add_agent(SimulatorRoutingAgent)

    pacman_agent.register_simulation_agent(simulator_agent)
    simulator_agent.register_pacman_agent(pacman_agent)

    system.run(sleep=0)