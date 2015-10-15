STATE = 'State'
ACTION = 'Action'
INIT = 'Init'
SAVE = 'Save'
ACK = 'Ack'

class BaseMessage(object):
    def __init__(self, msg_type=None):
        self.msg_type = msg_type


class StateMessage(BaseMessage):
    def __init__(self, msg_type=STATE, index=None, pacman_position=None,
        ghost_positions=None, food_positions=None, wall_positions=None,
        legal_actions=None, reward=None, executed_action=None, explore=None):
        super(StateMessage, self).__init__(msg_type=msg_type)
        self.index = index
        self.pacman_position = pacman_position
        self.ghost_positions = ghost_positions
        self.food_positions = food_positions
        self.wall_positions = wall_positions
        self.legal_actions = legal_actions
        self.reward = reward
        self.executed_action = executed_action
        self.explore = explore


class ActionMessage(BaseMessage):
    def __init__(self, msg_type=ACTION, index=None, action=None):
        super(ActionMessage, self).__init__(msg_type=msg_type)
        self.index = index
        self.action = action


class InitMessage(BaseMessage):
    def __init__(self, msg_type=INIT):
        super(InitMessage, self).__init__(msg_type=msg_type)


class SaveMessage(BaseMessage):
    def __init__(self, msg_type=SAVE, index=None, filename=None):
        super(SaveMessage, self).__init__(msg_type=msg_type)
        self.index = index
        self.filename = filename


class AckMessage(BaseMessage):
    def __init__(self, msg_type=ACK):
        super(AckMessage, self).__init__(msg_type=msg_type)