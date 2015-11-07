#!/usr/bin/env python

from __future__ import division
import communication as comm
import pickle
import agents
import messages
import state


class MessageRouter(object):
    def __init__(self):
        self.server = comm.Server()
        self.agents = {}
        self.agent_classes = {}
        self.agent_teams = {}
        self.game_states = {}

    def register_agent(self, message):
        print 'Initialized agent:', message.agent_id
        print 'Team:', message.agent_team
        print 'Type:', message.agent_class

        self.agent_classes[message.agent_id] = message.agent_class
        self.agent_teams[message.agent_id] = message.agent_team

    def get_agent_allies(self, agent_id):
        return [id_ for id_ in self.agent_teams
            if self.agent_teams[id_] == self.agent_teams[agent_id]
            and id_ != agent_id]

    def get_agent_enemies(self, agent_id):
        return [id_ for id_ in self.agent_teams
            if self.agent_teams[id_] != self.agent_teams[agent_id]
            and id_ != agent_id]

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

    def update_agent_state(self, state):
        agent_id = state.agent_id

        for id_, pos in state.agent_positions.items():
            self.game_states[agent_id].observe_agent(id_, pos)

    def choose_action(self, state):
        self.update_agent_state(state)
        agent_state = self.game_states[state.agent_id]
        agent_action = self.agents[state.agent_id].choose_action(agent_state, state.executed_action, state.reward, state.legal_actions, state.explore)
        agent_state.predict_agent(state.agent_id, agent_action)

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
                game_state = self.game_states[received_message.agent_id]
                game_state.set_walls(received_message.wall_positions)
                game_state.set_food_positions(received_message.food_positions)

                agent_action = self.choose_action(received_message)
                reply_message = self.create_action_message(received_message.agent_id, agent_action)
                self.send_message(reply_message)

                self.last_action = agent_action
            elif received_message.msg_type == messages.INIT:
                agent_id = received_message.agent_id
                ally_ids = self.get_agent_allies(agent_id)
                enemy_ids = self.get_agent_enemies(agent_id)
                print 'Allies:', self.get_agent_allies(agent_id)
                print 'Enemies:', self.get_agent_enemies(agent_id)
                self.agents[agent_id] = self.agent_classes[agent_id](ally_ids, enemy_ids)
                self.game_states[agent_id] = state.GameState(20, 11, [],
                    agent_id=agent_id, ally_ids=ally_ids, enemy_ids=enemy_ids)
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