#  -*- coding: utf-8 -*-

"""Code for communication between controller and simulator."""

import pickle
import zmq

# Default settings
DEFAULT_CLIENT_ADDRESS = 'localhost'
DEFAULT_TCP_PORT = 5555


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
        # The REP socket reads and saves all identity frames up to and
        # including the empty delimiter, then passes the following frame or
        # frames to the caller. REP sockets are synchronous and talk to one
        # peer at a time. If you connect a REP socket to multiple peers,
        # requests are read from peers in fair fashion, and replies are always
        # sent to the same peer that made the last request.
        # (http://zguide.zeromq.org/page:all#advanced-request-reply)
        super(ZMQServer, self).__init__(context, socket_type=zmq.REP)
        self.socket.bind(binding)


class ZMQClient(ZMQMessengerBase):
    """Inter-process communication server."""
    def __init__(self, context, connection):
        # The REQ socket sends, to the network, an empty delimiter frame in
        # front of the message data. REQ sockets are synchronous. REQ sockets
        # always send one request and then wait for one reply. REQ sockets talk
        # to one peer at a time. If you connect a REQ socket to multiple peers,
        # requests are distributed to and replies expected from each peer one
        # turn at a time.
        super(ZMQClient, self).__init__(context, socket_type=zmq.REQ)
        self.socket.connect(connection)


class InprocServer(ZMQServer):
    """Inter-process communication client."""
    def __init__(self, context, endpoint):
        binding = 'inproc://{}'.format(endpoint)
        super(InprocServer, self).__init__(context, binding)


class InprocClient(ZMQClient):
    """Inter-process communication client."""
    def __init__(self, context, endpoint):
        # If you're using inproc, both client and server must use the same
        # context instance, otherwise the connecting side will fail. Also, bind
        # first, connect later.
        # Reference: http://zguide.zeromq.org/php:chapter2
        connection = 'inproc://{}'.format(endpoint)
        super(InprocClient, self).__init__(context, connection)


class TCPServer(ZMQServer):
    """Inter-process communication client."""
    def __init__(self, port=DEFAULT_TCP_PORT):
        binding = 'tcp://*:{}'.format(port)
        super(TCPServer, self).__init__(zmq.Context(), binding)


class TCPClient(ZMQClient):
    """Inter-process communication client."""
    def __init__(self, address=DEFAULT_CLIENT_ADDRESS, port=DEFAULT_TCP_PORT):
        connection = 'tcp://{}:{}'.format(address, port)
        super(TCPClient, self).__init__(zmq.Context(), connection)
