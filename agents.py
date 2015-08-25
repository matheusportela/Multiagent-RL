import random
import learning


class PacmanAgent(object):
    """Pacman agent abstract class.

    Attributes:
        index: Pacman agent index for game referral.
    """
    def __init__(self):
        self.actions = ['North', 'South', 'East', 'West', 'Stop']

    def choose_action(self, state):
        """Select an action to be executed by the agent.

        Args:
            state: Current game state.

        Returns:
            A Direction for the agent to follow (NORTH, SOUTH, EAST, WEST or
            STOP).
        """
        raise NotImplementedError('Pacman agent must contain a choose_action method'
            'to select an action for the current game state.')


class GhostAgent(object):
    """Ghost agent abstract class.

    Attributes:
        index: Ghost agent index for game referral.
    """
    def __init__(self):
        self.actions = ['North', 'South', 'East', 'West']

    def choose_action(self, state):
        """Select an action to be executed by the agent.

        Ghosts can only select new actions at intersections or dead ends.

        Args:
            state: Current game state.

        Returns:
            A Direction for the agent to follow (NORTH, SOUTH, EAST, WEST or
            STOP).
        """
        raise NotImplementedError('Ghost agent must contain a choose_action method'
            'to select an action for the current game state.')


class RandomPacmanAgent(PacmanAgent):
    """Agent that randomly selects an action."""
    def choose_action(self, legal_actions):
        return random.choice(legal_actions)


class RandomGhostAgent(GhostAgent):
    """Agent that randomly selects an action."""
    def choose_action(self, legal_actions):
        return random.choice(legal_actions)


class LearningPacmanAgent(PacmanAgent):
    def __init__(self):
        super(LearningPacmanAgent, self).__init__()
        self.exploration_rate = 0.1
        self.learning = learning.QLearning(learning_rate=0.9, discount_factor=0.9,
            actions=self.actions)

    def choose_action(self, state, action, reward, legal_actions):
        self.learning.learn(state, action, reward)
        suggested_action = self.learning.act(state, legal_actions)

        if random.random() < self.exploration_rate:
            return random.choice(legal_actions)
        else:
            return suggested_action