import zmq
import time
import pickle

class Server(object):
    def __init__(self, port=5555):
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.bind('tcp://*:{}'.format(port))

    def recv(self):
        return self.socket.recv()

    def send(self, message):
        self.socket.send(message)


class Foo(object):
    def __init__(self):
        self.x = 100
        self.foo_y = 'HELLO, WORLD!'

    def __str__(self):
        return '{} {}'.format(self.x, self.foo_y)


if __name__ == '__main__':
    server = Server(port=5555)

    while True:
        request = server.recv()
        print 'Received request:', request

        message = Foo()
        print 'Sending message:', str(message)
        server.send(pickle.dumps(message))

        time.sleep(1)