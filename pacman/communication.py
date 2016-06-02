import zmq


class Server(object):
    def __init__(self, port=5555):
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.bind('tcp://*:{}'.format(port))

    def recv(self):
        return self.socket.recv()

    def send(self, message):
        self.socket.send(message)


class Client(object):
    def __init__(self, address='localhost', port=5555):
        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)
        self.socket.connect('tcp://{}:{}'.format(address, port))

    def recv(self):
        return self.socket.recv()

    def send(self, message):
        self.socket.send(message)