#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Routes messages between server and agents."""


from __future__ import division

import argparse

import messages
from state import GameState

# @todo properly include communication module from parent folder
import sys
sys.path.insert(0, '..')
import communication
import core

__author__ = "Matheus Portela and Guilherme N. Ramos"
__credits__ = ["Matheus Portela", "Guilherme N. Ramos", "Renato Nobre",
               "Pedro Saman"]
__maintainer__ = "Guilherme N. Ramos"
__email__ = "gnramos@unb.br"


def log(msg):
    print '[Controller] {}'.format(msg)


class Controller(core.BaseController):
    def __init__(self, *args, **kwargs):
        super(Controller, self).__init__(*args, **kwargs)

        self.agents = {}
        self.agent_classes = {}
        self.agent_teams = {}
        self.game_states = {}
        self.game_number = {}

    def start(self):
        log('Starting')

    def stop(self):
        log('Stopped')

    def is_finished(self):
        return False

    def step(self, msg):
        reply_msg = None

        if msg.type == messages.START_EXPERIMENT_MSG:
            reply_msg = self._start_experiment(msg)
        elif msg.type == messages.FINISH_EXPERIMENT_MSG:
            reply_msg = self._finish_experiment(msg)
        elif msg.type == messages.START_GAME_MSG:
            reply_msg = self._start_game(msg)
        elif msg.type == messages.FINISH_GAME_MSG:
            reply_msg = self._finish_game(msg)
        elif msg.type == messages.STATE_MSG:
            reply_msg = self._receive_state(msg)
        elif msg.type == messages.REWARD_MSG:
            reply_msg = self._receive_reward(msg)
        elif msg.type == messages.REQUEST_POLICY_MSG:
            reply_msg = self._send_policy_request(msg)
        elif msg.type == messages.POLICY_MSG:
            reply_msg = self._set_agent_policy(msg)
        else:
            raise ValueError('Unknown message type "{}"'.format(msg.type))

        return reply_msg

    def _start_experiment(self, msg):
        log('#{} Starting experiment for {}'.format(
            msg.agent_id, msg.agent_class.__name__))
        self.map_width = msg.map_width
        self.map_height = msg.map_height
        self.agent_classes[msg.agent_id] = msg.agent_class
        self.agent_teams[msg.agent_id] = msg.agent_team
        self.game_number[msg.agent_id] = 0
        return messages.AcknowledgementMessage()

    def _finish_experiment(self, msg):
        log('#{} Finishing experiment'.format(msg.agent_id))
        return messages.AcknowledgementMessage()

    def _start_game(self, msg):
        log('#{} Starting game'.format(msg.agent_id))
        agent_id = msg.agent_id
        ally_ids = self._get_allies(agent_id)
        enemy_ids = self._get_enemies(agent_id)

        if agent_id in self.agents:
            del self.agents[agent_id]
        self.agents[agent_id] = self.agent_classes[agent_id](agent_id,
                                                             ally_ids,
                                                             enemy_ids)

        return messages.AcknowledgementMessage(
            ally_ids=ally_ids, enemy_ids=enemy_ids)

    def _get_allies(self, agent_id):
        return [id_ for id_ in self.agent_teams
                if self.agent_teams[id_] == self.agent_teams[agent_id] and
                id_ != agent_id]

    def _get_enemies(self, agent_id):
        return [id_ for id_ in self.agent_teams
                if self.agent_teams[id_] != self.agent_teams[agent_id] and
                id_ != agent_id]

    def _finish_game(self, msg):
        log('#{} Finishing game'.format(msg.agent_id))

        self.game_number[msg.agent_id] += 1
        return messages.AcknowledgementMessage()

    def _receive_state(self, msg):
        log('#{} Receiving state'.format(msg.agent_id))

        choose_action = self.agents[msg.agent_id].choose_action
        agent_action = choose_action(msg.state, msg.executed_action,
                                     msg.reward, msg.legal_actions,
                                     msg.test_mode)

        log('#{} Sending action'.format(msg.agent_id))
        return messages.ActionMessage(agent_id=msg.agent_id,
                                      action=agent_action)

    def _receive_reward(self, msg):
        log('#{} Receiving reward'.format(msg.agent_id))
        reply_msg = messages.AcknowledgementMessage()
        return reply_msg

    def _send_policy_request(self, msg):
        log('#{} Sending policy'.format(msg.agent_id))
        policy = self.agents[msg.agent_id].get_policy()
        reply_message = messages.PolicyMessage(msg.agent_id, policy)
        return reply_message

    def _set_agent_policy(self, msg):
        log('#{} Receiving policy'.format(msg.agent_id))
        self.agents[msg.agent_id].set_policy(msg.policy)
        return messages.AcknowledgementMessage()


def build_controller(context=None, endpoint=None,
                     port=communication.DEFAULT_TCP_PORT):
    if context and endpoint:
        server = communication.InprocServer(context, endpoint)
        log('Connecting with inproc communication')
    else:
        server = communication.TCPServer(port)
        log('Connecting with TCP communication (port {})'.format(port))

    return Controller(server=server)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run Pac-Man controller system.')
    parser.add_argument('--port', dest='port', type=int,
                        default=communication.DEFAULT_TCP_PORT,
                        help='TCP port to connect to adapter')
    args, unknown = parser.parse_known_args()

    controller = build_controller(port=args.port)

    try:
        controller.run()
    except KeyboardInterrupt:
        print '\n\nInterrupted execution\n'
