import math


class Map(object):
    """Probabilistic map.

    Every cell contains a value in the interval [0, 1] indicating a probability.
    The entire map sums up to 1.
    """
    def __init__(self, width, height, walls=[]):
        self.width = width
        self.height = height
        self.action_to_pos = {
            'North': (1, 0),
            'South': (-1, 0),
            'East': (0, 1),
            'West': (0, -1),
            'Stop': (0, 0),
        }
        self.paths = None
        self.walls = walls
        self.cells = self.generate_cells()
        self.normalize()

    @property
    def walls(self):
        return self._walls

    @walls.setter
    def walls(self, walls):
        self._walls = walls
        self._calculate_all_paths()

    def __getitem__(self, i):
        return self.cells[i]

    def __setitem__(self, i, item):
        self.cells[i] = item

    def __iter__(self):
        for value in self.cells:
            yield value

    def __len__(self):
        return self.num_cells

    def __str__(self):
        string = []

        for y in range(self.height-1, -1, -1):
            for x in range(self.width):
                if self._is_wall((y, x)):
                    string.append('......')
                else:
                    string.append('%.4f' % self[y][x])
                string.append(' ')
            string.append('\n')

        return ''.join([str(line) for line in string])

    def _is_inbound(self, pos):
        return (0 <= pos[0] < self.height and 0 <= pos[1] < self.width)

    def _is_wall(self, pos):
        return (pos in self._walls)

    def _is_valid_position(self, pos):
        return (self._is_inbound(pos) and not self._is_wall(pos))

    def max(self):
        max_prob = float('-inf')

        for y in range(self.height):
            max_row = max(self[y])
            if max_row > max_prob:
                max_prob = max_row

        return max_prob

    def normalize(self):
        prob_sum = 0.0

        for x in range(self.width):
            for y in range(self.height):
                prob_sum += self[y][x]

        for x in range(self.width):
            for y in range(self.height):
                if self._is_wall((y, x)):
                    self[y][x] = 0.0
                elif prob_sum > 0:
                    prob = self[y][x] / prob_sum
                    self[y][x] = prob
                else:
                    self[y][x] = 1.0 / ((self.width * self.height) - len(self.walls))

    def generate_cells(self):
        cells = [[0 for _ in range(self.width)]
                    for _ in range(self.height)]
        return cells

    def get_maximum_position(self):
        max_position = (0, 0)
        max_prob = 0.0

        for x in range(self.width):
            for y in range(self.height):
                if self[y][x] > max_prob:
                    max_prob = self[y][x]
                    max_position = (y, x)

        return max_position

    def observe(self, pos, measurement_prob_dist_fn, *params):
        for x in range(self.width):
            for y in range(self.height):
                old_probability = self[y][x]
                new_probability = measurement_prob_dist_fn((y, x), pos, *params) * old_probability
                self[y][x] = new_probability

        self.normalize()

    def predict(self, action, action_prob_dist_fn, *params):
        cells = self.generate_cells()

        for x in range(self.width):
            for y in range(self.height):
                old_probability = self[y][x]

                for possible_action in self.action_to_pos:
                    next_y = y + self.action_to_pos[possible_action][0]
                    next_x = x + self.action_to_pos[possible_action][1]

                    if self._is_valid_position((next_y, next_x)):
                        action_probability = action_prob_dist_fn(action, possible_action, *params)
                        new_probability = action_probability * old_probability
                        cells[next_y][next_x] += new_probability

        self.cells = cells
        self.normalize()

    def _generate_next_pos(self, pos):
        next_pos = {}

        for action, delta in self.action_to_pos.items():
            candidate_pos = (pos[0] + delta[0], pos[1] + delta[1])

            if self._is_valid_position(candidate_pos):
                next_pos[candidate_pos] = action

        return next_pos

    def _calculate_paths(self, pos, max_distance=None):
        pos_to_path = {}
        current_pos = [pos]
        analyzed_pos = []

        while current_pos:
            p = current_pos.pop(0)
            analyzed_pos.append(p)

            if p in pos_to_path:
                path = pos_to_path[p]
            else:
                path = []

            next_pos = []
            for next_p, action in self._generate_next_pos(p).items():
                if next_p not in analyzed_pos:
                    next_pos.append(next_p)

                    if not max_distance or len(path) + 1 <= max_distance:
                        pos_to_path[next_p] = path + [action]
            current_pos.extend(next_pos)

        return pos_to_path

    def _calculate_all_paths(self, max_distance=None):
        paths = {}

        for y in range(self.height):
            for x in range(self.width):
                pos = (y, x)
                if self._is_valid_position(pos):
                    paths[pos] = self._calculate_paths(pos, max_distance=max_distance)

        self.paths = paths

    def calculate_distance(self, pos1, pos2):
        if self.paths == None:
            self._calculate_all_paths()

        if self._is_valid_position(pos1) and self._is_valid_position(pos2):
            if pos1 == pos2:
                return 0
            else:
                return len(self.paths[pos1][pos2])
        else:
            return float('inf')


def deterministic_distribution(action1, action2):
    if action1 == action2:
        return 1.0
    else:
        return 0.0

def semi_deterministic_distribution(action1, action2):
    if action1 == action2:
        return 0.99
    else:
        return 0.01

def gaussian_distribution(pos1, pos2, sd):
    diff_y = pos2[0] - pos1[0]
    diff_x = pos2[1] - pos1[1]
    return math.exp(-(diff_x**2 + diff_y**2) / (2 * sd**2))


class GameState(object):
    def __init__(self, width, height, walls):
        self.width = width
        self.height = height
        self.walls = walls
        self.agent_maps = {
            'pacman': Map(width, height, walls),
            'ghost': Map(width, height, walls),
        }
        self.food_map = None
        self.sd = 0.5

    def __str__(self):
        string = []

        for key, value in self.agent_maps.items():
            string.append(key)
            string.append(str(value))

        return '\n'.join(string)

    def set_food_positions(self, food_positions):
        if self.food_map == None:
            self.food_map = Map(self.width, self.height, self.walls)

            for x in range(self.width):
                for y in range(self.height):
                    if (y, x) in food_positions:
                        self.food_map[y][x] = 1.0
                    else:
                        self.food_map[y][x] = 0.0

    def set_walls(self, walls):
        for agent in self.agent_maps:
            if self.agent_maps[agent].walls == []:
                self.agent_maps[agent].walls = walls
                self.agent_maps[agent].normalize()

    def _observe_agent(self, agent, pos):
        self.agent_maps[agent].observe(pos, gaussian_distribution, self.sd)

    def observe_pacman(self, pos):
        self._observe_agent('pacman', pos)

    def observe_ghost(self, pos):
        # TODO: Should it be ghost_pos - pacman_pos?
        self._observe_agent('ghost', pos)

    def _get_agent_position(self, agent):
        return self.agent_maps[agent].get_maximum_position()

    def get_pacman_position(self):
        return self._get_agent_position('pacman')

    def get_ghost_position(self):
        return self._get_agent_position('ghost')

    def _predict_agent(self, agent, action):
        self.agent_maps[agent].predict(action, semi_deterministic_distribution)

    def predict_pacman(self, action):
        self._predict_agent('pacman', action)
        self._predict_food_positions()

    def predict_ghost(self, action):
        self._predict_agent('ghost', action)

    def _predict_food_positions(self):
        for x in range(self.width):
            for y in range(self.height):
                self.food_map[y][x] = self.food_map[y][x] * (1 - self.agent_maps['pacman'][y][x])

    def calculate_manhattan_distance(self, point1, point2):
        return (abs(point1[0] - point2[0]) + abs(point1[1] - point2[1]))

    def calculate_distance(self, point1, point2):
        return self.agent_maps['pacman'].calculate_distance(point1, point2)

    def get_food_distance(self):
        pacman_position = self.get_pacman_position()
        food_prob_threshold = self.food_map.max() / 2.0
        min_dist = float('inf')

        for x in range(self.width):
            for y in range(self.height):
                if self.food_map[y][x] > food_prob_threshold:
                    dist = self.calculate_distance(pacman_position, (y, x))

                    if dist < min_dist:
                        min_dist = dist

        return min_dist

    def get_ghost_distance(self):
        pacman_position = self.get_pacman_position()
        ghost_position = self.get_ghost_position()
        return self.calculate_distance(pacman_position, ghost_position)


if __name__ == '__main__':
    # X X _ _ _
    # . X o _ _
    # _ X _ _ _
    # _ X _ _ _
    # _ _ _ _ _

    walls = [(0, 0), (0, 1), (1, 1), (2, 1), (3, 1)]
    game_map = Map(10, 5, walls)
    action_to_pos = {
        'North': (1, 0),
        'South': (-1, 0),
        'East': (0, 1),
        'West': (0, -1),
        'Stop': (0, 0),
    }

    initial_pos = (1, 0)
    final_pos = (1, 2)

    print game_map
    positions = [(y, x) for y in range(game_map.height) for x in range(game_map.width)]
    for pos1 in positions:
        for pos2 in positions:
            print pos1, '->', pos2, game_map.calculate_distance(pos1, pos2)

    print game_map.calculate_distance((1, 0), (1, 0))