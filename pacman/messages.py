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
START_EXPERIMENT_MSG = 'Start Experiment'
FINISH_EXPERIMENT_MSG = 'Finish Experiment'
START_GAME_MSG = 'Start Game'
FINISH_GAME_MSG = 'Finish Game'
STATE_MSG = 'State'
ACTION_MSG = 'Action'
REWARD_MSG = 'Reward'


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


class AcknowledgementMessage(BaseMessage):
    """Simple acknowledgment message, used when no reply is required from the
    server.
    """
    def __init__(self):
        super(AcknowledgementMessage, self).__init__(type=ACK_MSG)


class StartExperimentMessage(BaseMessage):
    """Message announcing an experiment is about to start."""
    def __init__(self, agent_id, agent_team, agent_class, map_width,
                 map_height):
        super(StartExperimentMessage, self).__init__(
            type=START_EXPERIMENT_MSG, agent_id=agent_id,
            agent_team=agent_team, agent_class=agent_class,
            map_width=map_width, map_height=map_height)


class FinishExperimentMessage(BaseMessage):
    """Message announcing the experiment has just finished."""
    def __init__(self, agent_id):
        super(FinishExperimentMessage, self).__init__(
            type=FINISH_EXPERIMENT_MSG, agent_id=agent_id)


class StartGameMessage(BaseMessage):
    """Message announcing a game is about to start."""
    def __init__(self, agent_id):
        super(StartGameMessage, self).__init__(
            type=START_GAME_MSG, agent_id=agent_id)


class FinishGameMessage(BaseMessage):
    """Message announcing the game has just finished."""
    def __init__(self, agent_id):
        super(FinishGameMessage, self).__init__(
            type=FINISH_GAME_MSG, agent_id=agent_id)


class StateMessage(BaseMessage):
    """Message containing information on the current game state."""
    def __init__(self, agent_id, agent_positions, food_positions,
                 fragile_agents, wall_positions, legal_actions, reward,
                 executed_action, test_mode):
        super(StateMessage, self).__init__(
            type=STATE_MSG, agent_id=agent_id, agent_positions=agent_positions,
            food_positions=food_positions, fragile_agents=fragile_agents,
            wall_positions=wall_positions, legal_actions=legal_actions,
            reward=reward, executed_action=executed_action,
            test_mode=test_mode)


class ActionMessage(BaseMessage):
    """Message containing information on the selected action for the received
    state.
    """
    def __init__(self, agent_id, action):
        super(ActionMessage, self).__init__(
            type=ACTION_MSG, agent_id=agent_id, action=action)


class RewardMessage(BaseMessage):
    """Message containing the reward received after executing the action."""
    def __init__(self, agent_id, reward):
        super(RewardMessage, self).__init__(
            type=REWARD_MSG, agent_id=agent_id, reward=reward)


# =============================================================================
# ============================ Deprecated messages ============================
# =============================================================================

# Message types
BEHAVIOR_COUNT_MSG = 'BehaviorCount'
POLICY_MSG = 'Policy'
REQUEST_BEHAVIOR_COUNT_MSG = 'RequestBehaviorCount'
REQUEST_POLICY_MSG = 'RequestPolicy'


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


class RequestBehaviorCountMessage(BaseMessage):
    """Requests the identified agent's BaseMessage count information."""
    def __init__(self, agent_id):
        super(RequestBehaviorCountMessage, self).__init__(
            type=REQUEST_BEHAVIOR_COUNT_MSG, agent_id=agent_id)


class RequestPolicyMessage(BaseMessage):
    """Requests the identified agent's policy."""
    def __init__(self, agent_id):
        super(RequestPolicyMessage, self).__init__(
            type=REQUEST_POLICY_MSG, agent_id=agent_id)
