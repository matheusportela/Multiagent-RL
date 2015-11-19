STATE = 'State'
ACTION = 'Action'
INIT = 'Init'
START = 'Start'
REGISTER = 'Register'
ACK = 'Ack'
REQUEST_BEHAVIOR_COUNT = 'RequestBehaviorCount'
BEHAVIOR_COUNT = 'BehaviorCount'
REQUEST_POLICY = 'RequestPolicy'
POLICY = 'Policy'

class BaseMessage(object):
    def __init__(self, msg_type=None):
        self.msg_type = msg_type


class AckMessage(BaseMessage):
    def __init__(self):
        super(AckMessage, self).__init__(msg_type=ACK)


class StateMessage(BaseMessage):
    def __init__(self, agent_id=None, agent_positions=None, food_positions=None,
        fragile_agents=None, wall_positions=None, legal_actions=None,
        reward=None, executed_action=None, test_mode=None):
        super(StateMessage, self).__init__(msg_type=STATE)
        self.agent_id = agent_id
        self.agent_positions = agent_positions
        self.food_positions = food_positions
        self.fragile_agents = fragile_agents
        self.wall_positions = wall_positions
        self.legal_actions = legal_actions
        self.reward = reward
        self.executed_action = executed_action
        self.test_mode = test_mode


class ActionMessage(BaseMessage):
    def __init__(self, agent_id=None, action=None):
        super(ActionMessage, self).__init__(msg_type=ACTION)
        self.agent_id = agent_id
        self.action = action


class InitMessage(BaseMessage):
    def __init__(self, agent_id=None):
        super(InitMessage, self).__init__(msg_type=INIT)
        self.agent_id = agent_id


class StartMessage(BaseMessage):
    def __init__(self, agent_id=None, map_width=None, map_height=None):
        super(StartMessage, self).__init__(msg_type=START)
        self.agent_id = agent_id
        self.map_width = map_width
        self.map_height = map_height


class RegisterMessage(BaseMessage):
    def __init__(self, agent_id=None, agent_team=None, agent_class=None):
        super(RegisterMessage, self).__init__(msg_type=REGISTER)
        self.agent_id = agent_id
        self.agent_team = agent_team
        self.agent_class = agent_class


class RequestBehaviorCountMessage(BaseMessage):
    def __init__(self, agent_id=None):
        super(RequestBehaviorCountMessage, self).__init__(msg_type=REQUEST_BEHAVIOR_COUNT)
        self.agent_id = agent_id


class BehaviorCountMessage(BaseMessage):
    def __init__(self, count=None):
        super(BehaviorCountMessage, self).__init__(msg_type=BEHAVIOR_COUNT)
        self.count = count


class RequestPolicyMessage(BaseMessage):
    def __init__(self, agent_id=None):
        super(RequestPolicyMessage, self).__init__(msg_type=REQUEST_POLICY)
        self.agent_id = agent_id


class PolicyMessage(BaseMessage):
    def __init__(self, agent_id=None, policy=None):
        super(PolicyMessage, self).__init__(msg_type=POLICY)
        self.agent_id = agent_id
        self.policy = policy