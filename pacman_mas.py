#!/usr/bin/env python

from __future__ import division
import communication as comm
import pickle
import agents
import messages
import state


class MessageRouter(object):
    def __init__(self):
        self.agents = {}
        self.server = comm.Server()
        self.game_state = None

    def register_agent(self, message):
        print 'Initialized agent:', message.agent_id
        print 'Type:', message.agent_type
        print 'Parameters:', message.args, message.kwargs

        self.agents[message.agent_id] = message.agent_type(*message.args, **message.kwargs)

    def receive_message(self):
        message = pickle.loads(self.server.recv())
        print 'Received message:', message.__dict__
        return message

    def create_action_message(self, agent_id, action):
        message = messages.ActionMessage(agent_id=agent_id, action=action)
        return message

    def create_ack_message(self):
        message = messages.AckMessage()
        return message

    def create_behavior_count_message(self, agent_id):
        message = messages.BehaviorCountMessage(
            count=self.agents[agent_id].behavior_count)

        return message

    def reset_behavior_count(self, agent_id):
        self.agents[agent_id].reset_behavior_count()

    def send_message(self, message):
        print 'Sent message:', message.__dict__
        self.server.send(pickle.dumps(message))

    def generate_agent_state(self, state):
        self.game_state.observe_pacman(state.pacman_position)
        self.game_state.observe_ghosts(state.ghost_positions)
        return self.game_state

    def choose_action(self, state):
        # Agent state should be per agent, instead of a class attribute
        agent_state = self.generate_agent_state(state)
        agent_action = self.agents[state.agent_id].choose_action(agent_state, state.executed_action, state.reward, state.legal_actions, state.explore)
        if state.agent_id == 0:
            self.game_state.predict_pacman(agent_action)
        else:
            self.game_state.predict_ghost(state.agent_id - 1, agent_action)

        return agent_action

    def save_agent_policy(self, message):
        self.agents[message.agent_id].save_policy(message.filename)

    def load_agent_policy(self, message):
        self.agents[message.agent_id].load_policy(message.filename)

    def run(self):
        self.last_action = 'Stop'

        while True:
            received_message = self.receive_message()

            if received_message.msg_type == messages.STATE:
                self.game_state.set_walls(received_message.wall_positions)
                self.game_state.set_food_positions(received_message.food_positions)

                agent_action = self.choose_action(received_message)
                reply_message = self.create_action_message(received_message.agent_id, agent_action)
                self.send_message(reply_message)

                self.last_action = agent_action
            elif received_message.msg_type == messages.INIT:
                self.game_state = state.GameState(20, 11, [], num_ghosts=len(self.agents) - 1)
                self.send_message(self.create_ack_message())
            elif received_message.msg_type == messages.REGISTER:
                self.register_agent(received_message)
                self.send_message(self.create_ack_message())
            elif received_message.msg_type == messages.SAVE:
                self.save_agent_policy(received_message)
                self.send_message(self.create_ack_message())
            elif received_message.msg_type == messages.LOAD:
                self.load_agent_policy(received_message)
                self.send_message(self.create_ack_message())
            elif received_message.msg_type == messages.REQUEST_BEHAVIOR_COUNT:
                self.send_message(self.create_behavior_count_message(received_message.agent_id))
                self.reset_behavior_count(received_message.agent_id)

if __name__ == '__main__':
    router = MessageRouter()
    router.run()