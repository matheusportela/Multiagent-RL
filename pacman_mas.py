#!/usr/bin/env python

from __future__ import division
import communication as comm
import pickle
import agents
import messages


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
            message_type, message_data = pickle.loads(self.server.recv())

            if message_type == 'Positions':
                # agent_index, pacman_position, ghosts_position, legal_actions = message_data

                if message_data.index == 0:
                    agent_action = self.pacman_agent.choose_action(message_data.legal_actions)
                else:
                    agent_action = self.ghost_agents[message_data.index].choose_action(message_data.legal_actions)

                reply = (message_data.index, agent_action)
                self.server.send(pickle.dumps(reply))
            elif message_type == 'Begin':
                print 'Beginning game'

if __name__ == '__main__':
    num_ghosts = 4
    router = MessageRouter()
    router.register_pacman_agent(agents.LearningPacmanAgent())
    for _ in range(num_ghosts):
        router.register_ghost_agent(agents.RandomGhostAgent())
    router.run()