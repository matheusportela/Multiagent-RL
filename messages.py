STATE = 'State'
ACTION = 'Action'
INIT = 'Init'
SAVE = 'Save'
LOAD = 'Load'
ACK = 'Ack'
REQUEST_BEHAVIOR_COUNT = 'RequestBehaviorCount'
BEHAVIOR_COUNT = 'BehaviorCount'

class BaseMessage(object):
    def __init__(self, msg_type=None):
        self.msg_type = msg_type


class AckMessage(BaseMessage):
    def __init__(self, msg_type=ACK):
        super(AckMessage, self).__init__(msg_type=msg_type)


class StateMessage(BaseMessage):
    def __init__(self, msg_type=STATE, agent_id=None, pacman_position=None,
        ghost_positions=None, food_positions=None, wall_positions=None,
        legal_actions=None, reward=None, executed_action=None, explore=None):
        super(StateMessage, self).__init__(msg_type=msg_type)
        self.agent_id = agent_id
        self.pacman_position = pacman_position
        self.ghost_positions = ghost_positions
        self.food_positions = food_positions
        self.wall_positions = wall_positions
        self.legal_actions = legal_actions
        self.reward = reward
        self.executed_action = executed_action
        self.explore = explore


class ActionMessage(BaseMessage):
    def __init__(self, msg_type=ACTION, agent_id=None, action=None):
        super(ActionMessage, self).__init__(msg_type=msg_type)
        self.agent_id = agent_id
        self.action = action


class InitMessage(BaseMessage):
    def __init__(self, msg_type=INIT):
        super(InitMessage, self).__init__(msg_type=msg_type)


class SaveMessage(BaseMessage):
    def __init__(self, msg_type=SAVE, agent_id=None, filename=None):
        super(SaveMessage, self).__init__(msg_type=msg_type)
        self.agent_id = agent_id
        self.filename = filename


class LoadMessage(BaseMessage):
    def __init__(self, msg_type=LOAD, agent_id=None, filename=None):
        super(LoadMessage, self).__init__(msg_type=msg_type)
        self.agent_id = agent_id
        self.filename = filename


class RequestBehaviorCountMessage(BaseMessage):
    def __init__(self, msg_type=REQUEST_BEHAVIOR_COUNT, agent_id=None):
        super(RequestBehaviorCountMessage, self).__init__(msg_type=msg_type)
        self.agent_id = agent_id


class BehaviorCountMessage(BaseMessage):
    def __init__(self, msg_type=BEHAVIOR_COUNT, count=None):
        super(BehaviorCountMessage, self).__init__(msg_type=msg_type)
        self.count = count