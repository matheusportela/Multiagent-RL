#!/usr/bin/env python

from __future__ import division
import communication as comm
import pickle
import random


class RandomActionAgent(object):
    def __init__(self):
        self.actions = ['North', 'South', 'East', 'West', 'Stop']

    def choose_action(self, position):
        return random.choice(self.actions)


class MessageRouter(object):
    def __init__(self):
        self.pacman_agent = None
        self.ghost_agents = []
        self.server = comm.Server()

    def register_pacman_agent(self, agent):
        self.pacman_agent = agent

    def register_ghost_agent(self, agent):
        self.ghost_agents.append(agent)

    def run(self):
        while True:
            agent_index, agent_position = pickle.loads(self.server.recv())

            if agent_index == 0:
                agent_action = self.pacman_agent.choose_action(agent_position)
            else:
                agent_action = self.ghost_agents[agent_index].choose_action(agent_position)

            reply = (agent_index, agent_action)
            self.server.send(pickle.dumps(reply))

if __name__ == '__main__':
    router = MessageRouter()
    router.register_pacman_agent(RandomActionAgent())
    for _ in range(4):
        router.register_ghost_agent(RandomActionAgent())
    router.run()