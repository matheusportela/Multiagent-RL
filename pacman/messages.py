#!/usr/bin/env python
#  -*- coding: utf-8 -*-

"""Code for communication between controller and simulator."""

import pickle
import zmq

__author__ = "Matheus Portela and Guilherme N. Ramos"
__credits__ = ["Matheus Portela", "Guilherme N. Ramos", "Renato Nobre",
               "Pedro Saman"]
__maintainer__ = "Guilherme N. Ramos"
__email__ = "gnramos@unb.br"


###############################################################################
#                                  Messages                                   #
###############################################################################

# Message types
ACK_MSG = 'Acknowledgment'
ACTION_MSG = 'Action'
BEHAVIOR_COUNT_MSG = 'BehaviorCount'
POLICY_MSG = 'Policy'
REQUEST_REGISTER_MSG = 'RequestRegister'
REQUEST_BEHAVIOR_COUNT_MSG = 'RequestBehaviorCount'
REQUEST_GAME_START_MSG = 'RequestGameStart'
REQUEST_INIT_MSG = 'RequestInitialization'
REQUEST_POLICY_MSG = 'RequestPolicy'
STATE_MSG = 'State'


class BaseMessage(object):
    """Base class for Pac-Man messages."""
    def __init__(self, msg_type=None):
        self.__type = msg_type

    @property
    def type(self):
        return self.__type


class AckMessage(BaseMessage):
    """Simple acknowledgment message."""
    def __init__(self):
        super(AckMessage, self).__init__(msg_type=ACK_MSG)


class ActionMessage(BaseMessage):
    """Carries the information of an agent's action."""
    def __init__(self, agent_id=None, action=None):
        super(ActionMessage, self).__init__(msg_type=ACTION_MSG)

        self.agent_id = agent_id
        self.action = action


class BehaviorCountMessage(BaseMessage):
    """Carries the requested behavior count."""
    def __init__(self, count=None):
        super(BehaviorCountMessage, self).__init__(msg_type=BEHAVIOR_COUNT_MSG)

        self.count = count


class PolicyMessage(BaseMessage):
    """Carries the requested policy."""
    def __init__(self, agent_id=None, policy=None):
        super(PolicyMessage, self).__init__(msg_type=POLICY_MSG)

        self.agent_id = agent_id
        self.policy = policy


class RequestMessage(BaseMessage):
    """Requests some information."""
    def __init__(self, msg_type):
        super(RequestMessage, self).__init__(msg_type=msg_type)


class RequestInitializationMessage(RequestMessage):
    """Requests that the identified agent be REQUEST_INITialized."""
    def __init__(self, agent_id=None):
        super(RequestInitializationMessage,
              self).__init__(msg_type=REQUEST_INIT_MSG)

        self.agent_id = agent_id


class RequestBehaviorCountMessage(RequestMessage):
    """Requests the identified agent's RequestMessage count information."""
    def __init__(self, agent_id=None):
        super(RequestBehaviorCountMessage,
              self).__init__(msg_type=REQUEST_BEHAVIOR_COUNT_MSG)

        self.agent_id = agent_id


class RequestGameStartMessage(RequestMessage):
    """Requests that a game be started for the identified agent."""
    def __init__(self, agent_id=None, map_width=None, map_height=None):
        super(RequestGameStartMessage,
              self).__init__(msg_type=REQUEST_GAME_START_MSG)

        self.agent_id = agent_id
        self.map_width = map_width
        self.map_height = map_height


class RequestRegisterMessage(RequestMessage):
    """Requests that the identified agent (and associated information) be
    registered."""
    def __init__(self, agent_id=None, agent_team=None, agent_class=None):
        super(RequestRegisterMessage,
              self).__init__(msg_type=REQUEST_REGISTER_MSG)

        self.agent_id = agent_id
        self.agent_team = agent_team
        self.agent_class = agent_class


class RequestPolicyMessage(RequestMessage):
    """Requests the identified agent's policy."""
    def __init__(self, agent_id=None):
        super(RequestPolicyMessage, self).__init__(msg_type=REQUEST_POLICY_MSG)

        self.agent_id = agent_id


class StateMessage(BaseMessage):
    """Carries the information of a game state."""
    def __init__(self, agent_id=None, agent_positions=None,
                 food_positions=None, fragile_agents=None, wall_positions=None,
                 legal_actions=None, reward=None, executed_action=None,
                 test_mode=None):
        super(StateMessage, self).__init__(msg_type=STATE_MSG)

        self.agent_id = agent_id
        self.agent_positions = agent_positions
        self.food_positions = food_positions
        self.fragile_agents = fragile_agents
        self.wall_positions = wall_positions
        self.legal_actions = legal_actions
        self.reward = reward
        self.executed_action = executed_action
        self.test_mode = test_mode
