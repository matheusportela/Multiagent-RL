#!/usr/bin/env python
#  -*- coding: utf-8 -*-

"""Messages exchanged between controller and adapter."""


__author__ = "Matheus Portela and Guilherme N. Ramos"
__credits__ = ["Matheus Portela", "Guilherme N. Ramos", "Renato Nobre",
               "Pedro Saman"]
__maintainer__ = "Guilherme N. Ramos"
__email__ = "gnramos@unb.br"


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
    def __init__(self, type=None, **kwargs):
        self._type = type

        # Add all keyword arguments as instance attributes
        for attribute, value in kwargs.items():
            setattr(self, attribute, value)

    @property
    def type(self):
        return self._type


class AckMessage(BaseMessage):
    """Simple acknowledgment message."""
    def __init__(self):
        super(AckMessage, self).__init__(type=ACK_MSG)


class ActionMessage(BaseMessage):
    """Carries the information of an agent's action."""
    def __init__(self, agent_id, action):
        super(ActionMessage, self).__init__(
            type=ACTION_MSG, agent_id=agent_id, action=action)


class BehaviorCountMessage(BaseMessage):
    """Carries the requested behavior count."""
    def __init__(self, count):
        super(BehaviorCountMessage, self).__init__(
            type=BEHAVIOR_COUNT_MSG, count=count)


class PolicyMessage(BaseMessage):
    """Carries the requested policy."""
    def __init__(self, agent_id, policy):
        super(PolicyMessage, self).__init__(
            type=POLICY_MSG, agent_id=agent_id, policy=policy)


class RequestInitializationMessage(BaseMessage):
    """Requests that the identified agent be REQUEST_INITialized."""
    def __init__(self, agent_id):
        super(RequestInitializationMessage, self).__init__(
            type=REQUEST_INIT_MSG, agent_id=agent_id)

        self.agent_id = agent_id


class RequestBehaviorCountMessage(BaseMessage):
    """Requests the identified agent's BaseMessage count information."""
    def __init__(self, agent_id):
        super(RequestBehaviorCountMessage, self).__init__(
            type=REQUEST_BEHAVIOR_COUNT_MSG, agent_id=agent_id)


class RequestGameStartMessage(BaseMessage):
    """Requests that a game be started for the identified agent."""
    def __init__(self, agent_id, map_width, map_height):
        super(RequestGameStartMessage, self).__init__(
            type=REQUEST_GAME_START_MSG, agent_id=agent_id,
            map_width=map_width, map_height=map_height)


class RequestRegisterMessage(BaseMessage):
    """Requests that the identified agent (and associated information) be
    registered."""
    def __init__(self, agent_id, agent_team, agent_class):
        super(RequestRegisterMessage, self).__init__(
            type=REQUEST_REGISTER_MSG, agent_id=agent_id,
            agent_team=agent_team, agent_class=agent_class)


class RequestPolicyMessage(BaseMessage):
    """Requests the identified agent's policy."""
    def __init__(self, agent_id):
        super(RequestPolicyMessage, self).__init__(
            type=REQUEST_POLICY_MSG, agent_id=agent_id)


class StateMessage(BaseMessage):
    """Carries the information of a game state."""
    def __init__(self, agent_id, agent_positions, food_positions,
                 fragile_agents, wall_positions, legal_actions, reward,
                 executed_action, test_mode):
        super(StateMessage, self).__init__(
            type=STATE_MSG, agent_id=agent_id, agent_positions=agent_positions,
            food_positions=food_positions, fragile_agents=fragile_agents,
            wall_positions=wall_positions, legal_actions=legal_actions,
            reward=reward, executed_action=executed_action,
            test_mode=test_mode)
