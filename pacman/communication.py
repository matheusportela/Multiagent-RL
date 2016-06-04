#  -*- coding: utf-8 -*-
##    @package controller.py
#      @author Matheus Portela & Guilherme N. Ramos (gnramos@unb.br)
#
# Code for communication between controller and simulator.


import pickle
import zmq


# Default settings
DEFAULT_TCP_PORT = 5555
DEFAULT_CLIENT_ADDRESS = 'localhost'


class MessengerBase(object):
    """Base class for simple communicating messages."""
    def receive(self):
        """Requests a message and returns it."""
        raise NotImplementedError('Messenger class must receive messages.')

    def send(self, msg):
        """Sends the given message."""
        raise NotImplementedError('Messenger class must send messages.')

###############################################################################
###############################################################################
###############################################################################


class ZMQMessenger(MessengerBase):
    """Base class for simple communicating messages through zmq."""
    def __init__(self, port=DEFAULT_TCP_PORT):
        self.port = port
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
    """Communication server."""
    def __init__(self, port=DEFAULT_TCP_PORT):
        super(ZMQServer, self).__init__(port=port)

    def __configure_socket__(self):
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.bind('tcp://*:{}'.format(self.port))


class ZMQClient(ZMQMessenger):
    """Communication client."""
    def __init__(self, address=DEFAULT_CLIENT_ADDRESS, port=DEFAULT_TCP_PORT):
        self.address = address
        super(ZMQClient, self).__init__(port=port)

    def __configure_socket__(self):
        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)
        self.socket.connect('tcp://{}:{}'.format(self.address, self.port))
