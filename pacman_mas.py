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

    def receive_message(self):
        return pickle.loads(self.server.recv())

    def create_action_message(self, agent_index, action):
        message = messages.ActionMessage(
            msg_type=messages.ACTION,
            index=agent_index,
            action=action)
        return pickle.dumps(message)

    def send_message(self, message):
        self.server.send(message)

    def choose_action(self, state):
        agent_state = tuple([state.pacman_position] + [tuple([pos for pos in state.ghost_positions])] + [tuple(state.food_positions)])
        executed_action = self.last_action
        reward = state.score - self.previous_score

        if state.index == 0:
            agent_action = self.pacman_agent.choose_action(agent_state, executed_action, reward, state.legal_actions)
        else:
            agent_action = self.ghost_agents[state.index].choose_action(state.legal_actions)

        return agent_action

    def run(self):
        self.last_action = 'Stop'
        self.previous_score = 0

        while True:
            received_message = self.receive_message()

            if received_message.msg_type == messages.STATE:
                agent_action = self.choose_action(received_message)
                reply_message = self.create_action_message(received_message.index, agent_action)
                self.send_message(reply_message)

                self.previous_score = received_message.score
                self.last_action = agent_action

if __name__ == '__main__':
    num_ghosts = 4
    router = MessageRouter()
    router.register_pacman_agent(agents.LearningAgent())
    for _ in range(num_ghosts):
        router.register_ghost_agent(agents.RandomGhostAgent())
    router.run()