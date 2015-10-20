import math
import random
import learning
import pickle
import os.path


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

    def save_policy(self, filename):
        """Save the learned policy into filename.

        Args:
            filename: File which stores the policy data.
        """
        raise NotImplementedError('Pacman agent must be able to save its learned'
            'policy')

    def load_policy(self, filename):
        """Save the learned policy into filename.

        Args:
            filename: File which stores the policy data.
        """
        raise NotImplementedError('Pacman agent must be able to save its learned'
            'policy')


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


class QLearningAgent(PacmanAgent):
    def __init__(self):
        super(QLearningAgent, self).__init__()
        self.exploration_rate = 0.1
        self.learning = learning.QLearning(learning_rate=0.1, discount_factor=0.9,
            actions=self.actions)

    def choose_action(self, state, action, reward, legal_actions):
        self.learning.learn(state, action, reward)
        suggested_action = self.learning.act(state, legal_actions)

        if random.random() < self.exploration_rate:
            return random.choice(legal_actions)
        else:
            return suggested_action


class QLearningWithApproximationAgent(PacmanAgent):
    def __init__(self):
        super(QLearningWithApproximationAgent, self).__init__()
        self.features = [self.feature_ghost_distance, self.feature_food_distance]
        self.exploration_rate = 0.1
        self.learning = learning.QLearningWithApproximation(learning_rate=0.1,
            discount_factor=0.9, actions=self.actions, features=self.features)

    def _find_closest_distance(self, agent_position, position_list):
        closest_distance = float('inf')

        for position in position_list:
            distance = math.sqrt((agent_position[0] - position[0])**2 + (agent_position[1] - position[1])**2)
            if distance < closest_distance:
                closest_distance = distance

        return closest_distance

    def feature_ghost_distance(self, state, action):
        pacman_position = state[0]
        ghost_positions = state[1]
        return self._find_closest_distance(pacman_position, ghost_positions)

    def feature_food_distance(self, state, action):
        pacman_position = state[0]
        food_positions = state[2]
        return self._find_closest_distance(pacman_position, food_positions)

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
        self.features = [self.feature_ghost_distance, self.feature_food_distance]
        self.behaviors = [self.eat_food, self.flee_from_ghost, self.random]
        self.exploration_rate = 0.1
        self.learning = learning.QLearningWithApproximation(learning_rate=0.1,
            discount_factor=0.9, actions=self.behaviors, features=self.features,
            exploration_rate=self.exploration_rate)
        self.previous_behavior = self.behaviors[0]
        self.behavior_count = {}
        self.reset_behavior_count()

    def reset_behavior_count(self):
        for behavior in self.behaviors:
            self.behavior_count[behavior.__name__] = 0

    def save_policy(self, filename):
        with open(filename, 'w') as fout:
            pickle.dump(self.learning.get_weights(), fout)

    def load_policy(self, filename):
        if os.path.isfile(filename):
            with open(filename) as fin:
                self.learning.set_weights(pickle.load(fin))

    def calculate_manhattan_distance(self, point1, point2):
        return (abs(point1[0] - point2[0]) + abs(point1[1] - point2[1]))

    def feature_ghost_distance(self, state, action):
        pacman_position = state.get_pacman_position()
        ghost_position = state.get_ghost_position()
        distance = self.calculate_manhattan_distance(pacman_position, ghost_position)

        if distance == 0.0:
            distance = 1.0

        return (1.0/distance)

    def feature_food_distance(self, state, action):
        distance = state.get_food_distance()

        if distance == 0.0:
            distance = 1.0

        return (1.0/distance)

    def random(self, state):
        if self.legal_actions == []:
            return 'Stop'
        else:
            return random.choice(self.legal_actions)

    def eat_food(self, state):
        pacman_position = state.get_pacman_position()
        pacman_map = state.agent_maps['pacman']
        food_map = state.food_map
        food_prob_threshold = food_map.max() / 2.0
        best_action = None
        min_dist = None

        for action in self.legal_actions:
            diff = pacman_map.action_to_pos[action]
            new_position = (pacman_position[0] + diff[0], pacman_position[1] + diff[1])

            for x in range(food_map.width):
                for y in range(food_map.height):
                    new_distance = self.calculate_manhattan_distance(new_position, (y, x))

                    if (best_action == None) or (food_map[y][x] > food_prob_threshold and new_distance < min_dist):
                        min_dist = new_distance
                        best_action = action

        return best_action

    def flee_from_ghost(self, state):
        pacman_position = state.get_pacman_position()
        ghost_position = state.get_ghost_position()
        pacman_map = state.agent_maps['pacman']

        best_action = None
        max_distance = None

        for action in self.legal_actions:
            diff = pacman_map.action_to_pos[action]
            new_position = (pacman_position[0] + diff[0], pacman_position[1] + diff[1])
            new_distance = self.calculate_manhattan_distance(new_position, ghost_position)

            if (best_action == None) or (pacman_map._is_valid_position(new_position) and
                new_distance > max_distance):
                best_action = action
                max_distance = new_distance

        return best_action

    def choose_action(self, state, action, reward, legal_actions, explore):
        if explore:
            self.enable_exploration()
        else:
            self.disable_exploration()

        self.legal_actions = legal_actions
        self.learning.learn(state, self.previous_behavior, reward)
        behavior = self.learning.act(state, self.behaviors)
        self.previous_behavior = behavior
        suggested_action = behavior(state)

        self.behavior_count[behavior.__name__] += 1

        if suggested_action in legal_actions:
            return suggested_action
        elif legal_actions == []:
            return 'Stop'
        else:
            return random.choice(legal_actions)

    def enable_exploration(self):
        self.learning.exploration_rate = self.exploration_rate

    def disable_exploration(self):
        self.learning.exploration_rate = 0