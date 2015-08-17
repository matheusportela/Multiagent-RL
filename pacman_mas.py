#!/usr/bin/env python

from __future__ import division
from mas import pymas
import communication as comm


class PacmanAgent(pymas.Agent):
    def __init__(self):
        super(PacmanAgent, self).__init__()
        self.simulation_agent = None

    def register_simulation_agent(self, agent):
        self.simulation_agent = agent

    def send_action(self):
        msg = pymas.Message(receiver=self.simulation_agent.id, data='Pacman message')
        self.send_message(msg)

    def on_receive_message(self, message):
        print 'Received message: {}'.format(message)
        self.send_action()


class SimulatorRoutingAgent(pymas.Agent):
    def __init__(self):
        super(SimulatorRoutingAgent, self).__init__()
        self.pacman_agent = None
        self.server = comm.Server()

    def register_pacman_agent(self, agent):
        self.pacman_agent = agent

    def on_run(self):
        self.send_simulation_message()

    def send_simulation_message(self):
        msg = pymas.Message(receiver=self.pacman_agent.id, data='Simulator message')
        self.send_message(msg)

    def on_receive_message(self, message):
        print 'Received reply: {}'.format(message)
        self.server.recv()
        self.server.send('East')


if __name__ == '__main__':
    system = pymas.System()
    pacman_agent = system.add_agent(PacmanAgent)
    simulator_agent = system.add_agent(SimulatorRoutingAgent)

    pacman_agent.register_simulation_agent(simulator_agent)
    simulator_agent.register_pacman_agent(pacman_agent)

    system.run(sleep=0)