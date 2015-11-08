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
        print 'Registered %s\tID: %d\tClass: %s' % (message.agent_team, message.agent_id, message.agent_class.__name__)

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
        self.server.send(pickle.dumps(message))

    def update_agent_state(self, state):
        agent_id = state.agent_id

        for id_, pos in state.agent_positions.items():
            self.game_states[agent_id].observe_agent(id_, pos)

        for id_, status in state.fragile_agents.items():
            self.game_states[agent_id].observe_fragile_agent(id_, status)

    def choose_action(self, state):
        self.update_agent_state(state)
        agent_state = self.game_states[state.agent_id]
        agent_action = self.agents[state.agent_id].choose_action(agent_state, state.executed_action, state.reward, state.legal_actions, state.explore)

        for id_ in self.game_states:
            agent_state.predict_agent(id_, agent_action)

        return agent_action

    def create_policy_message(self, agent_id):
        policy = self.agents[agent_id].get_policy()
        return messages.PolicyMessage(agent_id=agent_id, policy=policy)

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

                if self.agent_teams[agent_id] == 'pacman':
                    eater = True
                else:
                    eater = False

                self.agents[agent_id] = self.agent_classes[agent_id](agent_id, ally_ids, enemy_ids)
                self.game_states[agent_id] = state.GameState(20, 11, [],
                    agent_id=agent_id, ally_ids=ally_ids, enemy_ids=enemy_ids,
                    eater=eater)
                self.send_message(self.create_ack_message())
            elif received_message.msg_type == messages.REGISTER:
                self.register_agent(received_message)
                self.send_message(self.create_ack_message())
            elif received_message.msg_type == messages.REQUEST_BEHAVIOR_COUNT:
                self.send_message(self.create_behavior_count_message(received_message.agent_id))
                self.reset_behavior_count(received_message.agent_id)
            elif received_message.msg_type == messages.REQUEST_POLICY:
                self.send_message(self.create_policy_message(received_message.agent_id))
            elif received_message.msg_type == messages.POLICY:
                self.agents[received_message.agent_id].set_policy(received_message.policy)
                self.send_message(self.create_ack_message())

if __name__ == '__main__':
    router = MessageRouter()

    try:
        router.run()
    except KeyboardInterrupt:
        print '\n\nInterrupted execution\n'