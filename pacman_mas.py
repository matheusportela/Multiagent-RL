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

    def generate_agent_state(self, state):
        # agent_state = tuple([state.pacman_position] + [tuple([pos for pos in state.ghost_positions])] + [tuple(state.food_positions)])
        # agent_state = tuple([state.pacman_position])

        # Food state indicates whether there is food in places relative to the
        # agent position.
        # Example:
        # Agent position = (3, 3)
        # Food position = (4, 4)
        # food_positions = [(0, 0), (0, 1), (1, 0), (1, 1), (0, -1), (-1, 0), (-1, -1)]
        # food_state = [False, False, False, True, False, False, False]
        food_positions = [(i, j) for i in range(-20, 20) for j in range(-20, 20)]
        food_state = [False] * len(food_positions)

        for food_position in state.food_positions:
            diff = (food_position[0] - state.pacman_position[0], food_position[1] - state.pacman_position[1])

            for i, position in enumerate(food_positions):
                if diff == position:
                    food_state[i] = True

        ghost_positions = [(i, j) for i in range(-20, 20) for j in range(-20, 20)]
        ghost_state = [False] * len(ghost_positions)

        for ghost_position in state.ghost_positions:
            diff = (ghost_position[0] - state.pacman_position[0], ghost_position[1] - state.pacman_position[1])

            for i, position in enumerate(ghost_positions):
                if diff == position:
                    ghost_state[i] = True

        agent_state = tuple([state.pacman_position] + ghost_state + food_state)
        return agent_state

    def choose_action(self, state):
        agent_state = self.generate_agent_state(state)

        if state.index == 0:
            agent_action = self.pacman_agent.choose_action(agent_state, state.executed_action, state.reward, state.legal_actions)
        else:
            agent_action = self.ghost_agents[state.index - 1].choose_action(state.legal_actions)

        return agent_action

    def run(self):
        self.last_action = 'Stop'

        while True:
            received_message = self.receive_message()

            if received_message.msg_type == messages.STATE:
                agent_action = self.choose_action(received_message)
                reply_message = self.create_action_message(received_message.index, agent_action)
                self.send_message(reply_message)

                self.last_action = agent_action

if __name__ == '__main__':
    num_ghosts = 1
    router = MessageRouter()
    router.register_pacman_agent(agents.QLearningAgent())
    for _ in range(num_ghosts):
        router.register_ghost_agent(agents.RandomGhostAgent())
    router.run()