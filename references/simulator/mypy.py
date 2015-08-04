from util import Stack, Queue, PriorityQueue
from game import Actions
from game import Directions
from copy import copy, deepcopy
from game import reconstituteGrid
from operator import sub

class Config(object):
    def __init__(self, pos, direction):
        self.pos = pos
        self.direction = direction

    def getDirection(self):
        return self.direction

def round_tuple(t):
    return (int(t[0]+0.5), int(t[1]+0.5))

def list_to_array(lis, width, height):
    return [[(x, y) in lis for x in range(width)] for y in range(height)]

def manhattan(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1]-p2[1])

class Problem(object):
    def data_structure(self):
        pass

    def push(self, value):
        pass

    def pop(self):
        pass

    def start_Value(self):
        pass

    def explore(self, value):
        pass

    def visit(self, value):
        pass

    def action(self, value):
        pass

    def is_goal(self, value):
        pass

    def goal_value(self, value, explored_nodes):
        pass

    def get_successors(self, value):
        pass

    def successor_value(self, current, successor):
        pass

    def fail_value(self):
        pass


class NPacmanMovesProblem(Problem):

    def __init__(self, game_state, n, blocks=None, position=None):
        self.ds = Stack()
        self.game_state = game_state
        self.position = position if position else self.game_state.getPacmanPosition()
        self.n = n
        ghosts = [search(GhostMovesProblem(round_tuple(ghost.getPosition()),ghost.scaredTimer, game_state, n)) for ghost in self.game_state.getGhostStates()]
        if blocks:
            self.blocks = blocks
        else:
            self.blocks = {}
            for i in range(n):
                self.blocks[i] = []
                for gn in ghosts:
                    self.blocks[i].extend([round_tuple(t) for t in gn[i]])
        #import ipdb; ipdb.set_trace()

    def data_structure(self):
        return self.ds

    def push(self, value):
        self.ds.push(value)

    def pop(self):
        return self.ds.pop()

    def get_start_state(self):
        foods = self.game_state.getFood()
        return (self.position, 0, foods)

    def start_value(self):
        return (self.get_start_state(), None, 0, None)

    def successor_value(self, current, successor):
        return (successor[0], successor[1], current[2] + 1, self.explore(current))

    def state(self, value):
        return value[0]

    def explore(self, value):
        return value[0][0]

    def parent(self, value):
        return value[3]

    def action(self, value):
        return value[1]

    def is_goal(self, value):
        state = self.state(value)
        return (state[1] == self.n) or (state[0] in self.game_state.getCapsules()) or (not state[2])

    def goal_value(self, value, explored_nodes):
        return False

    def get_successors(self, value):
        state = self.state(value)
        config = Config(self.explore(value), Directions.STOP)
        ghosts = self.blocks[state[1]]#[round_tuple(ghost.getPosition()) for ghost in self.game_state.getGhostStates() if ghost.scaredTimer <= 0]
        walls_grid = deepcopy(self.game_state.getWalls())
        walls = ghosts
        for w in walls:
            walls_grid[w[0]][w[1]] = True
        #import ipdb; ipdb.set_trace()
        #walls_grid = reconstituteGrid(walls)
        #list_to_array(walls, walls_grid.width, walls_grid.height)
        foods = self.game_state.getFood()

        actions = Actions.getPossibleActions(config, walls_grid)
        result = []
        for action in actions:
            new_state = []
            position = Actions.getSuccessor(state[0], action)
            fs = copy(foods)
            if position in fs:
                fs.remove(position)
            result.append(((position, state[1] + 1, fs), action))
        return result

    def fail_value(self):
        return True


class GhostMovesProblem(Problem):

    def __init__(self, position, scaredTimer, game_state, n):
        self.ds = Queue()
        self.position = position
        self.scaredTimer = scaredTimer
        self.game_state = game_state
        self.n = n
        self.visited = {}
        for i in range(n):
            self.visited[i] = []
    #
    def data_structure(self):
        return self.ds

    #
    def push(self, value):
        self.ds.push(value)

    #
    def pop(self):
        return self.ds.pop()

    def get_start_state(self):
        return (self.position, 0)

    #
    def start_value(self):
        return (self.get_start_state(), None, 0, None, self.scaredTimer)

    #
    def successor_value(self, current, successor):
        return (successor[0], successor[1], current[2] + 1, self.explore(current), max(0, self.scaredTimer - 1))

    #
    def explore(self, value):
        return (value[0][0], value[4] == 0)

    #
    def visit(self, value):
        e = self.explore(value)
        if e[1]:
            for i in range(value[2], self.n):
                self.visited[i].append(e[0])

    def state(self, value):
        return value[0]
        
    #
    def action(self, value):
        return value[2] 

    #
    def is_goal(self, value):
        state = self.state(value)
        return (state[1] == self.n)

    #
    def goal_value(self, value, explored_nodes):
        return self.visited

    #
    def get_successors(self, value):
        state = self.state(value)
        config = Config(state[0], Directions.STOP)
        walls_grid = self.game_state.getWalls()
        
        actions = Actions.getPossibleActions(config, walls_grid)
        result = []
        for action in actions:
            position = Actions.getSuccessor(state[0], action)
            result.append(((position, state[1] - 1), action))
        return result

    #
    def fail_value(self):
        return self.visited

class AStartMazeSearchProblem(Problem):

    def __init__(self, position, position2, blocks, game_state):
        self.ds = PriorityQueue()
        self.position = round_tuple(position)
        self.position2 = round_tuple(position2)
        self.blocks = [round_tuple(t) for t in blocks]
        self.game_state = game_state
        self.antecessor = {}

    #
    def data_structure(self):
        return self.ds

    #
    def push(self, value):
        self.ds.push(value, value[2] + self.heuristic(value))

    def heuristic(self, value):
        return abs(value[0][0] - self.position2[0]) + abs(value[0][1] - self.position2[1])

    #
    def pop(self):
        return self.ds.pop()

    def get_start_state(self):
        return self.position

    #
    def start_value(self):
        return (self.get_start_state(), None, 0, None)

    #
    def successor_value(self, current, successor):
        return (successor[0], successor[1], current[2] + 1, self.explore(current))

    #
    def explore(self, value):
        return value[0]

    #
    def visit(self, value):
        self.antecessor[value[0]] = value[3]

    def state(self, value):
        return value[0]
        
    #
    def action(self, value):
        return value[1] 

    #
    def is_goal(self, value):
        state = self.state(value)
        return (state == self.position2)

    #
    def goal_value(self, value, explored_nodes):
        result = []
        current = value[0]
        while current != self.position:
            direction = explored_nodes[current]
            result.append(direction)
            current = self.antecessor[current]
        result.reverse()
        return (len(result), result)

    #
    def get_successors(self, value):
        state = self.state(value)
        config = Config(self.explore(value), Directions.STOP)
        walls_grid = deepcopy(self.game_state.getWalls())
        walls = self.blocks
        for w in walls:
            walls_grid[w[0]][w[1]] = True

        actions = Actions.getPossibleActions(config, walls_grid)
        result = []
        for action in actions:
            position = Actions.getSuccessor(state, action)
            result.append((position, action))
        return result

    #
    def fail_value(self):
        return (-1, [Directions.STOP])

class NearestProblem(Problem):

    def __init__(self, position, blocks, game_state, elements):
        self.elements = elements
        self.ds = PriorityQueue()
        self.position = round_tuple(position)
        self.blocks = [round_tuple(t) for t in blocks]
        self.game_state = game_state
        self.antecessor = {}

    #
    def data_structure(self):
        return self.ds

    #
    def push(self, value):
        self.ds.push(value, value[2] + self.heuristic(value))

    def heuristic(self, value):
        if not self.elements:
            return 0
        return min(manhattan(value[0], element) for element in self.elements)

    #
    def pop(self):
        return self.ds.pop()

    def get_start_state(self):
        return self.position

    #
    def start_value(self):
        return (self.get_start_state(), None, 0, None)

    #
    def successor_value(self, current, successor):
        return (successor[0], successor[1], current[2] + 1, self.explore(current))

    #
    def explore(self, value):
        return value[0]

    #
    def visit(self, value):
        self.antecessor[value[0]] = value[3]

    def state(self, value):
        return value[0]
        
    #
    def action(self, value):
        return value[1] 

    #
    def is_goal(self, value):
        state = self.state(value)
        return (state in self.elements) or not self.elements

    #
    def goal_value(self, value, explored_nodes):
        if not self.elements:
            return self.fail_value()
        result = []
        current = value[0]
        while current != self.position:
            direction = explored_nodes[current]
            result.append(direction)
            current = self.antecessor[current]
        result.reverse()
        return (len(result), result)

    #
    def get_successors(self, value):
        state = self.state(value)
        config = Config(self.explore(value), Directions.STOP)
        walls_grid = deepcopy(self.game_state.getWalls())
        walls = self.blocks
        for w in walls:
            walls_grid[w[0]][w[1]] = True

        actions = Actions.getPossibleActions(config, walls_grid)
        result = []
        for action in actions:
            position = Actions.getSuccessor(state, action)
            result.append((position, action))
        return result

    #
    def fail_value(self):
        return (-1, [Directions.STOP])

class NearestFoodProblem(NearestProblem):

    def __init__(self, position, blocks, game_state):
        NearestProblem.__init__(self, position, blocks, game_state, game_state.getFood())


class NearestCapsuleProblem(NearestProblem):

    def __init__(self, position, blocks, game_state):
        NearestProblem.__init__(self, position, blocks, game_state, game_state.getCapsules())

class NearestWhiteGhostProblem(NearestProblem):

    def __init__(self, position, blocks, game_state):
        NearestProblem.__init__(self, position, blocks, game_state, [round_tuple(ghost.getPosition()) for ghost in game_state.getGhostStates() if ghost.scaredTimer])

class NearestColoredGhostProblem(NearestProblem):

    def __init__(self, position, blocks, game_state):
        NearestProblem.__init__(self, position, blocks, game_state, [round_tuple(ghost.getPosition()) for ghost in game_state.getGhostStates() if not ghost.scaredTimer])


def search(problem):
    problem.push(problem.start_value())
    explored_nodes = {}
    current = None
    while not problem.data_structure().isEmpty():
        current = problem.pop()
        explore = problem.explore(current)
        if explore not in explored_nodes:
            problem.visit(current)
            explored_nodes[explore] = problem.action(current)
            if problem.is_goal(current):
                return problem.goal_value(current, explored_nodes)
            successors = problem.get_successors(current)
            for successor in successors:
                problem.push(problem.successor_value(current, successor))
    return problem.fail_value()

def near(x, y):
    nx, ny = map(sub, x, y)
    return (abs(nx) == 1 and ny == 0) or (abs(ny) == 1 and nx == 0)

def apply_filter(filt, current, height, width, multiply=None):
    if not multiply:
        multiply = current
    tiles = [[0 for y in xrange(height)] for x in xrange(width)]
    for x in xrange(1, width - 1):
        for y in xrange(1, height -1):
            summ = 0
            for j, dx in enumerate(xrange(-1, 2)):
                for i, dy in enumerate(xrange(-1, 2)):
                    cx = x + dx
                    cy = y + dy
                    summ += filt[j][i]*int(current[cx][cy])
            tiles[x][y] = summ*int(multiply[x][y])
    return tiles

def make_groups(tiles):
    groups = []
    for tile in tiles:
        group_append = [tile]
        for group in copy(groups):
            if any(near(f, tile) for f in group):
                group_append += group
                groups.remove(group)
        groups.append(tuple(group_append))
    return groups

def invert_grid(grid):
    new_grid = deepcopy(grid)
    for x in range(grid.width):
        for y in range(grid.height):
            new_grid[x][y] = not new_grid[x][y]
    return new_grid

def grid_to_graph(state , grid):
    
    dilate = [
        [0, 1, 0],
        [1, 0, 1],
        [0, 1, 0],
    ]
    find_vertices = [
        [0, -1, 0],
        [-1, 2, -1],
        [0, -1, 0],
    ]
    
    currentState = state
    height = grid.height
    width = grid.width

    data = deepcopy(grid.data)
    #starting_vertex = currentState.getPacmanPosition()
    #data[starting_vertex[0]][starting_vertex[1]] = True
    vertices = apply_filter(find_vertices, data, height, width)
    vertices_list = set((x, y) for y in xrange(height) for x in xrange(width) if vertices[x][y])
    paths_list = [tile for tile in grid.asList() if not tile in vertices_list] 
    return (paths_list, vertices_list, None)
    #vertices[starting_vertex[0]][starting_vertex[1]] = True
    temp = vertices
    wrong_vertices = apply_filter(dilate, vertices, height, width)
    vertices_list = set((x, y) for y in xrange(height) for x in xrange(width) if vertices[x][y] and not wrong_vertices[x][y])

    fat_paths_plus_new_vertices = apply_filter(dilate, wrong_vertices, height, width, grid)
    fat_paths_plus_new_vertices = [[1 if fat_paths_plus_new_vertices[x][y] else 0 for y in xrange(height)] for x in xrange(width)]
    

    new_vertices = apply_filter(find_vertices, fat_paths_plus_new_vertices, height, width)
    fat_paths = set((x, y) for y in xrange(height) for x in xrange(width) if fat_paths_plus_new_vertices[x][y] and not new_vertices[x][y] > 0)
    
    new_vertices_list = set((x, y) for y in xrange(height) for x in xrange(width) if new_vertices[x][y]>0)
    vertices_list = vertices_list.union(new_vertices_list)

    #vertices_list.add(starting_vertex)
    #paths_list = [tile for tile in grid.asList() if not tile in vertices_list] 
       
    

    return (paths_list, vertices_list, fat_paths)

def make_edges(gameState, paths_list, vertices_list):

    #group paths
    groups = make_groups(paths_list)

    #i->[path tile]
    paths = dict((i, []) for i, _ in enumerate(groups))
    #vertex -> [path index]
    vertex_paths = dict((vertex, []) for vertex in vertices_list)

    for i, group in enumerate(groups):
        for vertex in vertices_list:
            if any(near(group_tile, vertex) for group_tile in group):
                paths[i].append(vertex)
                vertex_paths[vertex].append(i)

    for vertex in vertices_list:
        for vert2 in vertices_list:
            if near(vert2, vertex):
                s1 = set(vertex_paths[vertex])
                s2 = set(vertex_paths[vert2])
                if not len(s1.intersection(s2)):
                    groups.append(tuple())
                    pos = len(groups) - 1
                    paths[pos] = []
                    paths[pos].append(vertex)
                    paths[pos].append(vert2)
                    vertex_paths[vertex].append(pos)
                    vertex_paths[vert2].append(pos)

    height = gameState.getWalls().height
    width = gameState.getWalls().width
    

    edges = dict((v, {}) for v in vertices_list)
    for group, vertices in paths.items():
        v0 = vertices[0]
        cost = len(groups[group]) + 1
        if len(vertices) == 1:
            if not v0 in edges[v0]:
                edges[v0][v0] = []
            edges[v0][v0].append((group, cost))
        else:
            v1 = vertices[1]
            if not v1 in edges[v0]:
                edges[v0][v1] = []
            if not v0 in edges[v1]:
                edges[v1][v0] = []
            edges[v0][v1].append((group, cost))
            edges[v1][v0].append((group, cost))
                           
        
    return (groups, paths, vertex_paths, edges)

def nearest_distances(state):
    ghosts = state.getGhostStates()
    ghost_positions = [round_tuple(ghost.getPosition()) for ghost in ghosts if not ghost.scaredTimer]
    
    pos = state.getPacmanPosition()
        
    nearest_ghost_distance = search(
        NearestColoredGhostProblem(pos, [], state)
    )[0]

    nearest_capsule_distance = search(
        NearestCapsuleProblem(pos, ghost_positions, state)
    )[0]

    nearest_white_ghost = search(
        NearestWhiteGhostProblem(pos, ghost_positions+state.getCapsules(), state)
    )[0]

    nearest_food = search(
        NearestFoodProblem(pos, ghost_positions, state)
    )[0]

    return (nearest_ghost_distance, nearest_capsule_distance, nearest_white_ghost, nearest_food)