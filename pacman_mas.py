#!/usr/bin/env python

from __future__ import division
import communication as comm
import pickle
import agents
import messages
import state


class MessageRouter(object):
    def __init__(self):
        self.pacman_agent = None
        self.ghost_agents = []
        self.server = comm.Server()
        self.game_state = state.GameState(20, 11, [])

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

        # food_positions = [(i, j) for i in range(-20, 20) for j in range(-20, 20)]
        # food_state = [False] * len(food_positions)

        # for food_position in state.food_positions:
        #     diff = (food_position[0] - state.pacman_position[0], food_position[1] - state.pacman_position[1])

        #     for i, position in enumerate(food_positions):
        #         if diff == position:
        #             food_state[i] = True

        self.game_state.observe_pacman(state.pacman_position)
        self.game_state.observe_ghost(state.ghost_positions[0])

        # agent_state = tuple([self.game_state.get_pacman_position(), self.game_state.get_ghost_position(), self.game_state.get_food_distance()])
        # return agent_state
        return self.game_state

    def choose_action(self, state):
        agent_state = self.generate_agent_state(state)

        if state.index == 0:
            agent_action = self.pacman_agent.choose_action(agent_state, state.executed_action, state.reward, state.legal_actions, state.explore)
            self.game_state.predict_pacman(agent_action)
        else:
            agent_action = self.ghost_agents[state.index - 1].choose_action(state.legal_actions)
            self.game_state.predict_ghost(agent_action)

        return agent_action

    def save_agent_policy(self, message):
        if message.index == 0:
            self.pacman_agent.save_policy(message.filename)
        else:
            self.ghost_agents[state.index - 1].save_policy(message.filename)

    def run(self):
        self.last_action = 'Stop'

        while True:
            print 'Waiting for message'
            received_message = self.receive_message()
            print received_message

            if received_message.msg_type == messages.STATE:
                self.game_state.set_walls(received_message.wall_positions)
                self.game_state.set_food_positions(received_message.food_positions)

                agent_action = self.choose_action(received_message)
                reply_message = self.create_action_message(received_message.index, agent_action)
                self.send_message(reply_message)

                self.last_action = agent_action
            elif received_message.msg_type == messages.INIT:
                self.game_state = state.GameState(20, 11, [])
                message = pickle.dumps(messages.InitMessage(msg_type=messages.INIT))
                self.send_message(message)
                # print self.game_state.food_map
                # print self.game_state.get_ghost_distance()
                # print self.game_state.get_food_distance()
            elif received_message.msg_type == messages.SAVE:
                print 'Received save message'
                self.save_agent_policy(received_message)
                print 'Saved policy'
                self.send_message(messages.AckMessage())
                print 'Sent ack message'

if __name__ == '__main__':
    num_ghosts = 1
    router = MessageRouter()
    router.register_pacman_agent(agents.BehaviorLearningAgent())
    for _ in range(num_ghosts):
        router.register_ghost_agent(agents.RandomGhostAgent())
    router.run()