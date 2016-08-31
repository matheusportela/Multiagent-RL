#  -*- coding: utf-8 -*-
##     @package controller.py
#      @author Matheus Portela & Guilherme N. Ramos (gnramos@unb.br)
#
# Code for communication between controller and simulator.


import pickle
import zmq


# Default settings
DEFAULT_CLIENT_ADDRESS = 'localhost'
DEFAULT_TCP_PORT = 5555


###############################################################################
#                                Messengers                                   #
###############################################################################
class ZMQMessengerBase(object):
    """Base class for simple communicating messages through zmq."""
    def __init__(self, context, socket_type):
        self.socket = context.socket(socket_type)

    def receive(self):
        """Requests a message and returns it."""
        return pickle.loads(self.socket.recv())

    def send(self, msg):
        """Sends the given message."""
        self.socket.send(pickle.dumps(msg))


class ZMQServer(ZMQMessengerBase):
    """Inter-process communication server."""
    def __init__(self, context, binding):
        super(ZMQServer, self).__init__(context, socket_type=zmq.REP)
        self.socket.bind(binding)
        # http://zguide.zeromq.org/page:all#advanced-request-reply
        # The REP socket reads and saves all identity frames up to and
        # including the empty delimiter, then passes the following frame or
        # frames to the caller. REP sockets are synchronous and talk to one
        # peer at a time. If you connect a REP socket to multiple peers,
        # requests are read from peers in fair fashion, and replies are always
        # sent to the same peer that made the last request.


class ZMQClient(ZMQMessengerBase):
    """Inter-process communication server."""
    def __init__(self, context, connection):
        super(ZMQClient, self).__init__(context, socket_type=zmq.REQ)
        self.socket.connect(connection)
        # The REQ socket sends, to the network, an empty delimiter frame in
        # front of the message data. REQ sockets are synchronous. REQ sockets
        # always send one request and then wait for one reply. REQ sockets talk
        # to one peer at a time. If you connect a REQ socket to multiple peers,
        # requests are distributed to and replies expected from each peer one
        # turn at a time.


class TCPServer(ZMQServer):
    """Inter-process communication client."""
    def __init__(self, address=DEFAULT_CLIENT_ADDRESS, port=DEFAULT_TCP_PORT):
        binding = 'tcp://*:{}'.format(port)
        super(TCPServer, self).__init__(zmq.Context(), binding)


class TCPClient(ZMQClient):
    """Inter-process communication client."""
    def __init__(self, address=DEFAULT_CLIENT_ADDRESS, port=DEFAULT_TCP_PORT):
        connection = 'tcp://{}:{}'.format(address, port)
        super(TCPClient, self).__init__(zmq.Context(), connection)


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
