STATE = 'State'
ACTION = 'Action'
INIT = 'Init'
REGISTER = 'Register'
SAVE = 'Save'
LOAD = 'Load'
ACK = 'Ack'
REQUEST_BEHAVIOR_COUNT = 'RequestBehaviorCount'
BEHAVIOR_COUNT = 'BehaviorCount'

class BaseMessage(object):
    def __init__(self, msg_type=None):
        self.msg_type = msg_type


class AckMessage(BaseMessage):
    def __init__(self):
        super(AckMessage, self).__init__(msg_type=ACK)


class StateMessage(BaseMessage):
    def __init__(self, agent_id=None, pacman_position=None,
        ghost_positions=None, food_positions=None, wall_positions=None,
        legal_actions=None, reward=None, executed_action=None, explore=None):
        super(StateMessage, self).__init__(msg_type=STATE)
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
    def __init__(self, agent_id=None, action=None):
        super(ActionMessage, self).__init__(msg_type=ACTION)
        self.agent_id = agent_id
        self.action = action


class InitMessage(BaseMessage):
    def __init__(self):
        super(InitMessage, self).__init__(msg_type=INIT)


class RegisterMessage(BaseMessage):
    def __init__(self, agent_id=None, agent_team=None, agent_class=None,
        args=[], kwargs={}):
        super(RegisterMessage, self).__init__(msg_type=REGISTER)
        self.agent_id = agent_id
        self.agent_team = agent_team
        self.agent_class = agent_class
        self.args = args
        self.kwargs = kwargs


class SaveMessage(BaseMessage):
    def __init__(self, agent_id=None, filename=None):
        super(SaveMessage, self).__init__(msg_type=SAVE)
        self.agent_id = agent_id
        self.filename = filename


class LoadMessage(BaseMessage):
    def __init__(self, agent_id=None, filename=None):
        super(LoadMessage, self).__init__(msg_type=LOAD)
        self.agent_id = agent_id
        self.filename = filename


class RequestBehaviorCountMessage(BaseMessage):
    def __init__(self, agent_id=None):
        super(RequestBehaviorCountMessage, self).__init__(msg_type=REQUEST_BEHAVIOR_COUNT)
        self.agent_id = agent_id


class BehaviorCountMessage(BaseMessage):
    def __init__(self, count=None):
        super(BehaviorCountMessage, self).__init__(msg_type=BEHAVIOR_COUNT)
        self.count = count