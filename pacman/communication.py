#  -*- coding: utf-8 -*-
##    @package controller.py
#      @author Matheus Portela & Guilherme N. Ramos (gnramos@unb.br)
#
# Code for communication between controller and simulator.


import zmq


# Default settings
DEFAULT_TCP_PORT = 5555
DEFAULT_CLIENT_ADDRESS = 'localhost'


class Server(object):
    def __init__(self, port=DEFAULT_TCP_PORT):
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.bind('tcp://*:{}'.format(port))

    def recv(self):
        return self.socket.recv()

    def send(self, message):
        self.socket.send(message)


class Client(object):
    def __init__(self, address=DEFAULT_CLIENT_ADDRESS, port=DEFAULT_TCP_PORT):
        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)
        self.socket.connect('tcp://{}:{}'.format(address, port))

    def recv(self):
        return self.socket.recv()

    def send(self, message):
        self.socket.send(message)
