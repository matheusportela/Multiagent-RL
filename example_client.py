import zmq
import pickle

class Client(object):
    def __init__(self, address='localhost', port=5555):
        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)
        self.socket.connect('tcp://{}:{}'.format(address, port))

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
    client = Client(port=5555)

    for i in range(10):
        print 'Sending request:', i
        client.send('{}'.format(i))

        message = pickle.loads(client.recv())
        print 'Received reply:', str(message)