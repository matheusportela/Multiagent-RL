#!/usr/bin/env python

from __future__ import division
import communication as comm
import pickle
import agents
import messages
import state


class MessageRouter(object):
    def __init__(self, num_ghosts=1):
        self.num_ghosts = num_ghosts
        self.pacman_agent = None
        self.ghost_agents = []
        self.server = comm.Server()
        self.game_state = state.GameState(20, 11, [], num_ghosts=num_ghosts)

    def register_pacman_agent(self, agent):
        self.pacman_agent = agent

    def register_ghost_agent(self, agent):
        self.ghost_agents.append(agent)

    def receive_message(self):
        message = pickle.loads(self.server.recv())
        print 'Received message:', message.__dict__
        return message

    def create_action_message(self, agent_id, action):
        message = messages.ActionMessage(
            msg_type=messages.ACTION,
            agent_id=agent_id,
            action=action)
        return message

    def create_ack_message(self):
        message = messages.AckMessage()
        return message

    def create_behavior_count_message(self, agent_id):
        if agent_id == 0:
            message = messages.BehaviorCountMessage(
                count=self.pacman_agent.behavior_count)
        else:
            message = messages.BehaviorCountMessage(
                count=self.ghost_agents[agent_id - 1].behavior_count)

        return message

    def reset_behavior_count(self, agent_id):
        if agent_id == 0:
            self.pacman_agent.reset_behavior_count()
        else:
            self.ghost_agents[agent_id - 1].reset_behavior_count()

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

        if state.agent_id == 0:
            # agent_action = self.pacman_agent.choose_action(agent_state, state.executed_action, state.reward, state.legal_actions, state.explore)
            agent_action = self.pacman_agent.choose_action(state.legal_actions)
            self.game_state.predict_pacman(agent_action)
        else:
            agent_action = self.ghost_agents[state.agent_id - 1].choose_action(state.legal_actions)
            self.game_state.predict_ghost(state.agent_id - 1, agent_action)

        return agent_action

    def save_agent_policy(self, message):
        if message.agent_id == 0:
            self.pacman_agent.save_policy(message.filename)
            print 'Pacman saved policy:', self.pacman_agent.learning.weights
        else:
            self.ghost_agents[state.agent_id - 1].save_policy(message.filename)

    def load_agent_policy(self, message):
        if message.agent_id == 0:
            self.pacman_agent.load_policy(message.filename)
            print 'Pacman loaded policy:', self.pacman_agent.learning.weights
        else:
            self.ghost_agents[state.agent_id - 1].load_policy(message.filename)

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
                self.game_state = state.GameState(20, 11, [], num_ghosts=num_ghosts)
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
    num_ghosts = 2
    router = MessageRouter(num_ghosts=num_ghosts)
    # router.register_pacman_agent(agents.BehaviorLearningAgent(num_ghosts))
    router.register_pacman_agent(agents.RandomPacmanAgent())
    for _ in range(num_ghosts):
        router.register_ghost_agent(agents.RandomGhostAgent())
    router.run()