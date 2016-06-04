#  -*- coding: utf-8 -*-
##    @package controller.py
#      @author Matheus Portela & Guilherme N. Ramos (gnramos@unb.br)
#
# Code for communication between controller and simulator.


import pickle
import zmq


# Default settings
DEFAULT_IPC_PIPE = 'ipc:///tmp/multiagent-rl.pipe'
DEFAULT_CLIENT_ADDRESS = 'localhost'
DEFAULT_TCP_PORT = 5555


class ZMQMessenger(object):
    """Base class for simple communicating messages through zmq."""
    def __init__(self):
        self.__configure_socket__()

    def __configure_socket__(self):
        raise NotImplementedError('ZMQMessenger must configure socket.')

    def receive(self):
        """Requests a message and returns it."""
        return pickle.loads(self.socket.recv())

    def send(self, msg):
        """Sends the given message."""
        self.socket.send(pickle.dumps(msg))


class ZMQServer(ZMQMessenger):
    """Inter-process communication server."""
    def __init__(self, binding=DEFAULT_IPC_PIPE):
        self.binding = binding
        super(ZMQServer, self).__init__()

    def __configure_socket__(self):
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.bind(self.binding)


class ZMQClient(ZMQMessenger):
    """Inter-process communication client."""
    def __init__(self, connection=DEFAULT_IPC_PIPE):
        self.connection = connection
        super(ZMQClient, self).__init__()

    def __configure_socket__(self):
        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)
        self.socket.connect(self.connection)


class TCPServer(ZMQServer):
    """Inter-process communication server."""
    def __init__(self, port=DEFAULT_TCP_PORT):
        binding = 'tcp://*:{}'.format(port)
        super(TCPServer, self).__init__(binding)


class TCPClient(ZMQClient):
    """Inter-process communication client."""
    def __init__(self, address=DEFAULT_CLIENT_ADDRESS, port=DEFAULT_TCP_PORT):
        connection = 'tcp://{}:{}'.format(address, port)
        super(TCPClient, self).__init__(connection)
