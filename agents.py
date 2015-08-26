import math
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


class BehaviorLearningAgent(PacmanAgent):
    def __init__(self):
        super(BehaviorLearningAgent, self).__init__()
        self.behaviors = [self.eat_food, self.flee_from_ghost, self.hunt_ghost]
        # self.behaviors = [self.hunt_ghost]
        self.exploration_rate = 0.1
        self.learning = learning.QLearning(learning_rate=0.9, discount_factor=0.9,
            actions=self.behaviors)
        self.previous_behavior = self.behaviors[0]

    def _find_closest(self, agent_position, position_list):
        closest_positions = []
        closest_distance = float('inf')

        for position in position_list:
            distance = math.sqrt((agent_position[0] - position[0])**2 + (agent_position[1] - position[1])**2)
            if distance < closest_distance:
                closest_distance = distance
                closest_positions = [position]
            elif distance == closest_distance:
                closest_positions.append(position)

        return random.choice(closest_positions)

    def eat_food(self, state):
        pacman_position = state[0]
        food_positions = state[2]

        closest_food_position = self._find_closest(pacman_position, food_positions)

        diff = (closest_food_position[0] - pacman_position[0],
                closest_food_position[1] - pacman_position[1])
        angle = math.atan2(diff[1], diff[0])

        if -math.pi/4.0 <= angle < math.pi/4.0:
            action = 'East'
        elif math.pi/4.0 <= angle < 3*math.pi/4.0:
            action = 'North'
        elif 3*math.pi/4.0 <= angle < math.pi or -math.pi <= angle < -3*math.pi/4.0:
            action = 'West'
        elif -3*math.pi/4.0 <= angle < math.pi/4.0:
            action = 'South'
        else:
            action = 'Stop'

        return action

    def flee_from_ghost(self, state):
        pacman_position = state[0]
        ghost_positions = state[1]

        closest_ghost_position = self._find_closest(pacman_position, ghost_positions)

        diff = (closest_ghost_position[0] - pacman_position[0],
                closest_ghost_position[1] - pacman_position[1])
        angle = math.atan2(diff[1], diff[0])

        if -math.pi/4.0 <= angle < math.pi/4.0:
            action = 'West'
        elif math.pi/4.0 <= angle < 3*math.pi/4.0:
            action = 'South'
        elif 3*math.pi/4.0 <= angle < math.pi or -math.pi <= angle < -3*math.pi/4.0:
            action = 'East'
        elif -3*math.pi/4.0 <= angle < math.pi/4.0:
            action = 'North'
        else:
            action = 'Stop'

        return action

    def hunt_ghost(self, state):
        pacman_position = state[0]
        ghost_positions = state[1]

        closest_ghost_position = self._find_closest(pacman_position, ghost_positions)

        diff = (closest_ghost_position[0] - pacman_position[0],
                closest_ghost_position[1] - pacman_position[1])
        angle = math.atan2(diff[1], diff[0])

        if -math.pi/4.0 <= angle < math.pi/4.0:
            action = 'East'
        elif math.pi/4.0 <= angle < 3*math.pi/4.0:
            action = 'North'
        elif 3*math.pi/4.0 <= angle < math.pi or -math.pi <= angle < -3*math.pi/4.0:
            action = 'West'
        elif -3*math.pi/4.0 <= angle < math.pi/4.0:
            action = 'South'
        else:
            action = 'Stop'

        return action

    def choose_action(self, state, action, reward, legal_actions):
        self.learning.learn(state, self.previous_behavior, reward)
        behavior = self.learning.act(state, self.behaviors)
        suggested_action = behavior(state)

        if suggested_action in legal_actions:
            return suggested_action
        else:
            return random.choice(legal_actions)