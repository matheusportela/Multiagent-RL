import game
import util
import random

class PacmanAgent(game.Agent):
    """Pacman agent abstract class.

    Attributes:
        index: Pacman agent index for game referral.
    """

    def __init__(self):
        self.index = 0

    def getAction(self, state):
        """Select an action to be executed by the agent.

        Args:
            state: Current game state.

        Returns:
            A Direction for the agent to follow (NORTH, SOUTH, EAST, WEST or
            STOP).
        """
        raise NotImplementedError('Pacman agent must contain a getAction method'
            'to select an action for the current game state.')


class GhostAgent(game.Agent):
    """Ghost agent abstract class.

    Attributes:
        index: Ghost agent index for game referral.
    """

    def __init__(self, index):
        self.index = index

    def getAction(self, state):
        """Select an action to be executed by the agent.

        Ghosts can only select new actions at intersections or dead ends.

        Args:
            state: Current game state.

        Returns:
            A Direction for the agent to follow (NORTH, SOUTH, EAST, WEST or
            STOP).
        """
        raise NotImplementedError('Ghost agent must contain a getAction method'
            'to select an action for the current game state.')    


class RandomPacmanAgent(PacmanAgent):
    """Agent that randomly selects an action."""
    def getAction(self, state):
        actions = state.getLegalPacmanActions()
        return random.choice(actions)


class RandomGhostAgent(GhostAgent):
    """Agent that randomly selects an action."""
    def __init__(self, index):
        self.index = index

    def getAction(self, state):
        actions = state.getLegalActions(self.index)
        return random.choice(actions)