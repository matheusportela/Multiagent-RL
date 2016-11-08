import communication


class BaseAdapterAgent(object):
    """Base class for an intelligent agent, specific to the experiment adapter.
    """
    def __init__(self, client=None):
        """Constructor.

        Parameters:
        client -- Client object from communication module.
        """

        # Communication between adapter and controller
        if client:
            self.client = client
        else:
            self.client = communication.TCPClient()

        # Whether the agent is learning from current experiences or should only
        # exploit what has been learned previously.
        self.is_learning = True

    def set_learning(self, is_learning):
        """Set whether the agent is learning from experiences.

        Parameters:
        is_learning -- Boolean defining whether the agent is learning or not.
        """
        self.is_learning = is_learning

    def communicate(self, message):
        """Send message to controller and wait for its reply.

        Parameters:
        message -- Message to be sent to controller.

        Returns:
        Message received from controller.
        """
        self.send(message)
        return self.receive()

    def receive(self):
        """Receive message from controller.

        Returns:
        Message received from controller.
        """
        return self.client.receive()

    def send(self, message):
        """Send message to controller.

        Parameters:
        message -- Message to be sent to controller.
        """
        self.client.send(message)

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


class BaseControllerAgent(object):
    """Autonomous agent for game controller."""
    def __init__(self, agent_id):
        self.agent_id = agent_id

    def get_policy(self):
        """Get the current action selection policy."""
        return None

    def set_policy(self, policy):
        """Set an action selection policy."""
        pass

    def learn(self, state, action, reward):
        """Learn from received reward after executing an action.

        Parameters:
        state -- Current game state.
        action -- Last executed action.
        reward -- Reward for the previous action.
        """
        raise NotImplementedError

    def act(self, state, legal_actions):
        """Return an action to be executed by the agent.

        Parameters:
        state -- Current game state.
        legal_actions -- List of currently allowed actions.

        Returns:
        An action to be executed.
        """
        raise NotImplementedError


class BaseController(object):
    """Base class for a controller, which manages agents and routes messages.
    """
    def __init__(self, server=None):
        """Constructor.

        Parameters:
        server -- Server object from communication module.
        """

        # Communication between controller and adapter
        if server:
            self.server = server
        else:
            self.server = communication.TCPServer()

    def log(self, string):
        """Log message to standard output.

        Parameters:
        string -- String to output.
        """
        print '[Controller] {}'.format(string)

    def communicate(self):
        """Send message to adapter and wait for its reply.

        Parameters:
        message -- Message to be sent to adapter.

        Returns:
        Message received from adapter.
        """
        message = self.receive()
        reply_message = self.step(message)
        self.send(reply_message)

    def receive(self):
        """Receive message from adapter.

        Returns:
        Message received from adapter.
        """
        return self.server.receive()

    def send(self, message):
        """Send message to controller.

        Parameters:
        message -- Message to be sent to adapter.
        """
        self.server.send(message)

    def step(self, message):
        """Implement logic to handle income messages.

        Parameters:
        message -- Message received from adapter.

        Returns:
        Message to be sent to adapter.
        """
        raise NotImplementedError

    def start(self):
        """Execute when starting the controller."""
        raise NotImplementedError

    def stop(self):
        """Execute when stopping the controller."""
        raise NotImplementedError

    def is_finished(self):
        """Whether the experiment has finished.

        Returns:
        Boolean indicating whether the experiment has finished.
        """
        raise NotImplementedError

    def run(self):
        """Execute controller until experiment has finished."""
        self.start()
        while not self.is_finished():
            self.communicate()
        self.stop()


class BaseExperiment(object):
    """Base class for an experiment, which is specific to the adapter."""
    def __init__(self, agents=[], learn_games=0, test_games=0):
        """Constructor.

        Parameters:
        agents -- List of agents for this experiment.
        learn_games -- Number of games to be executed in learning phase.
        test_games -- Number of games to be executed in testing phase.
        """
        self.agents = []
        self.learn_games = learn_games
        self.test_games = test_games
        self.game_number = 0

    @property
    def is_learn_game(self):
        """Whether the experiment is in learning phase.

        Returns:
        Boolean indicating the current game is a learning game.
        """
        return self.game_number < self.learn_games

    @property
    def is_test_game(self):
        """Whether the experiment is in testing phase.

        Returns:
        Boolean indicating the current game is a testing game.
        """
        return self.game_number >= self.learn_games

    def run(self):
        """Execute experiment."""
        self.start()
        self._start_experiment()
        self._execute()
        self._finish_experiment()
        self.stop()

    def start(self):
        """Execute before the experiment."""
        pass

    def _start_experiment(self):
        """Start experiment for each agent."""
        for agent in self.agents:
            agent.start_experiment()

    def _execute(self):
        """Execute experiment."""
        self._execute_games(self.learn_games)

        for agent in self.agents:
            agent.set_learning(False)

        self._execute_games(self.test_games)

    def _execute_games(self, num_games):
        """Execute games.

        Parameters:
        num_games -- Number of games to be executed.
        """
        for _ in xrange(num_games):
            for agent in self.agents:
                agent.start_game()

            self.execute_game()

            self.game_number += 1

            for agent in self.agents:
                agent.finish_game()

    def execute_game(self):
        """Execute game."""
        raise NotImplementedError

    def _finish_experiment(self):
        """Finish experiment for each agent."""
        for agent in self.agents:
            agent.finish_experiment()

    def stop(self):
        """Execute after the experiment."""
        pass
