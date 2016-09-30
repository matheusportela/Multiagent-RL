#  -*- coding: utf-8 -*-
##    @package controller.py
#      @author Matheus Portela & Guilherme N. Ramos (gnramos@unb.br)
#
# Routes messages between server and agents.


from __future__ import division

from berkeley.game import Directions

import agents
import cliparser
import messages
from state import GameState

## @todo properly include communication module from parent folder
import sys
sys.path.insert(0, '..')
from communication import ZMQServer
def log(msg):
    print '[Controller] {}'.format(msg)


class Controller(ZMQServer):
    """Keeps the agent states and controls messages to clients."""
    def __init__(self, context, binding):
        super(Controller, self).__init__(context, binding)

        self.agents = {}
        self.agent_classes = {}
        self.agent_teams = {}
        self.game_states = {}
        self.game_number = {}

        log('Ready!')

    def __choose_action__(self, state):
        # Update agent state
        for id_, pos in state.agent_positions.items():
            self.game_states[state.agent_id].observe_agent(id_, pos)

        for id_, status in state.fragile_agents.items():
            self.game_states[state.agent_id].observe_fragile_agent(id_, status)

        # Choose action
        agent_state = self.game_states[state.agent_id]
        choose_action = self.agents[state.agent_id].choose_action
        agent_action = choose_action(agent_state, state.executed_action,
                                     state.reward, state.legal_actions,
                                     state.test_mode)

        for id_ in self.game_states:
            agent_state.predict_agent(id_, agent_action)

        return agent_action

    def __get_allies__(self, agent_id):
        return [id_ for id_ in self.agent_teams
                if self.agent_teams[id_] == self.agent_teams[agent_id] and
                id_ != agent_id]

    def __get_enemies__(self, agent_id):
        return [id_ for id_ in self.agent_teams
                if self.agent_teams[id_] != self.agent_teams[agent_id] and
                id_ != agent_id]

    def __initialize_agent__(self, msg):
        agent_id = msg.agent_id
        ally_ids = self.__get_allies__(agent_id)
        enemy_ids = self.__get_enemies__(agent_id)

        if agent_id in self.agents:
            del self.agents[agent_id]

        self.game_number[agent_id] = 0
        self.agents[agent_id] = self.agent_classes[agent_id](agent_id,
                                                             ally_ids,
                                                             enemy_ids)
        log('Initialized {} #{}'.format(self.agent_teams[agent_id], agent_id))

        reply_msg = messages.AckMessage()
        self.send(reply_msg)

    def __register_agent__(self, msg):
        self.agent_classes[msg.agent_id] = msg.agent_class
        self.agent_teams[msg.agent_id] = msg.agent_team

        log('Registered {} #{}'.format(msg.agent_class.__name__, msg.agent_id))

        reply_msg = messages.AckMessage()
        self.send(reply_msg)

    def __reply__(self, msg):
        if msg.type == messages.STATE_MSG:
            self.last_action = self.__send_agent_action__(msg)
        elif msg.type == messages.REQUEST_INIT_MSG:
            self.__initialize_agent__(msg)
        elif msg.type == messages.REQUEST_GAME_START_MSG:
            self.__start_game_for_agent__(msg)
            self.game_number[msg.agent_id] += 1
        elif msg.type == messages.REQUEST_REGISTER_MSG:
            self.__register_agent__(msg)
        elif msg.type == messages.REQUEST_BEHAVIOR_COUNT_MSG:
            self.__request_behavior_count__(msg.agent_id)
        elif msg.type == messages.REQUEST_POLICY_MSG:
            self.__send_policy_request__(msg)
        elif msg.type == messages.POLICY_MSG:
            self.__set_agent_policy__(msg)

    def __request_behavior_count__(self, agent_id):
        count = self.agents[agent_id].behavior_count
        reply_msg = messages.BehaviorCountMessage(count)
        self.send(reply_msg)

        self.agents[agent_id].reset_behavior_count()

    def __send_agent_action__(self, msg):
        game_state = self.game_states[msg.agent_id]
        ## @todo is it necessary to set walls every time?
        game_state.set_walls(msg.wall_positions)
        game_state.set_food_positions(msg.food_positions)

        agent_action = self.__choose_action__(msg)
        reply_msg = messages.ActionMessage(msg.agent_id, agent_action)
        self.send(reply_msg)

        return agent_action

    def __send_policy_request__(self, msg):
        policy = self.agents[msg.agent_id].get_policy()
        reply_message = messages.PolicyMessage(msg.agent_id, policy)
        self.send(reply_message)

    def __set_agent_policy__(self, msg):
        if msg.policy:
            self.agents[msg.agent_id].set_policy(msg.policy)
        self.send(messages.AckMessage())

    def __start_game_for_agent__(self, msg):
        ally_ids = self.__get_allies__(msg.agent_id)
        enemy_ids = self.__get_enemies__(msg.agent_id)

        eater = (self.agent_teams[msg.agent_id] == 'pacman')

        if msg.agent_id in self.game_states:
            del self.game_states[msg.agent_id]

        iteration = self.game_number[msg.agent_id]
        self.game_states[msg.agent_id] = GameState(width=msg.map_width,
                                                   height=msg.map_height,
                                                   walls=[],
                                                   agent_id=msg.agent_id,
                                                   ally_ids=ally_ids,
                                                   enemy_ids=enemy_ids,
                                                   eater=eater,
                                                   iteration=iteration)

        reply_msg = messages.AckMessage()
        self.send(reply_msg)
        log('Start game for {} #{}'.format(self.agent_teams[msg.agent_id],
                                           msg.agent_id))

    def run(self):
        log('Now running')

        self.last_action = Directions.STOP

        while True:
            msg = self.receive()
            self.__reply__(msg)

if __name__ == '__main__':
    try:
        controller = cliparser.get_Controller()
        controller.run()
    except KeyboardInterrupt:
        print '\n\nInterrupted execution\n'
