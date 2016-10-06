import communication


class BaseAgent(object):
    def __init__(self, client=communication.TCPClient()):
        super(BaseAgent, self).__init__()
        self.client = client

    def receive(self):
        return self.client.receive()

    def send(self, message):
        self.client.send(message)

    def communicate(self, message):
        self.send(message)
        return self.receive()

    def start_game(self):
        raise NotImplementedError

    def finish_game(self):
        raise NotImplementedError

    def learn(self):
        raise NotImplementedError

    def act(self):
        raise NotImplementedError


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
