import communication


class BaseAdapterAgent(object):
    def __init__(self, client=None):
        # Communication between adapter and controller
        if client:
            self.client = client
        else:
            self.client = communication.TCPClient()

        # Whether the agent must can explore the action space or must only
        # exploit the best actions so far
        self.is_exploring = True

    def receive(self):
        """Receive message from controller."""
        return self.client.receive()

    def send(self, message):
        """Send message to controller."""
        self.client.send(message)

    def communicate(self, message):
        """Send message to controller and wait for its reply."""
        self.send(message)
        return self.receive()

    def set_explore(self, is_exploring):
        """Set whether the agent can explore the action space."""
        self.is_exploring = is_exploring

    def start_experiment(self):
        """Execute before starting an experiment."""
        raise NotImplementedError

    def finish_experiment(self):
        """Execute after finishing an experiment."""
        raise NotImplementedError

    def start_game(self):
        """Execute before starting a game episode."""
        raise NotImplementedError

    def finish_game(self):
        """Execute after finishing a game episode."""
        raise NotImplementedError

    def send_state(self):
        """Send state message from adapter to controller."""
        raise NotImplementedError

    def receive_action(self):
        """Receive selected action from controller."""
        raise NotImplementedError

    def send_reward(self):
        """Send reward message from adapter to controller."""
        raise NotImplementedError


class BaseController(object):
    def __init__(self, server=None):
        # Communication between controller and adapter
        if server:
            self.server = server
        else:
            self.server = communication.TCPServer()

    def log(self, msg):
        print '[Controller] {}'.format(msg)

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


class BaseExperiment(object):
    def start(self):
        raise NotImplementedError

    def execute(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def run(self):
        self.start()
        self.execute()
        self.stop()
