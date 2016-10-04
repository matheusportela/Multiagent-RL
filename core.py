import communication


class BaseController(object):
    def __init__(self, server=None):
        super(BaseController, self).__init__()

        if server:
            self.server = server
        else:
            self.server = communication.TCPServer()

    def receive(self):
        return self.server.receive()

    def send(self, message):
        self.server.send(message)

    def communicate(self):
        raise NotImplementedError

    def start(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def step(self, action):
        raise NotImplementedError

    def is_finished(self):
        raise NotImplementedError

    def run(self):
        self.start()
        while not self.is_finished():
            self.communicate()
            self.step()
        self.stop()
