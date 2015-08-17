# multiAgents.py
# --------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

from util import manhattanDistance
from util import Stack, Queue
from copy import copy
from game import Directions
import random, util
from game import Actions
from game import Agent
import time
from mypy import NPacmanMovesProblem, AStartMazeSearchProblem, GhostMovesProblem, NearestFoodProblem, search, round_tuple
from mypy import grid_to_graph, invert_grid, make_edges, NearestCapsuleProblem
from mypy import NearestWhiteGhostProblem, NearestColoredGhostProblem, nearest_distances

class ReflexAgent(Agent):
    """
        A reflex agent chooses an action at each choice point by examining
        its alternatives via a state evaluation function.

        The code below is provided as a guide.  You are welcome to change
        it in any way you see fit, so long as you don't touch our method
        headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {North, South, West, East, Stop}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        "*** YOUR CODE HERE ***"
        #print "successorGameState", successorGameState
        #print "newPos", newPos
        #print "newGhostStates", newGhostStates
        #print "newFood", newFood
        ghost_distances = [manhattanDistance(ghostState.getPosition(), newPos) for ghostState in newGhostStates]
        nearest_ghost_distance = min(ghost_distances)
        delta_score = successorGameState.getScore() - currentGameState.getScore() 

        foods = newFood.asList()
        food_distances = [manhattanDistance(food, newPos) for food in foods]
        nearest_food_distance = min(food_distances) if food_distances else 0


        
        #import ipdb; ipdb.set_trace()
        return delta_score + nearest_ghost_distance - nearest_food_distance

def scoreEvaluationFunction(currentGameState):
    """
        This default evaluation function just returns the score of the state.
        The score is the same one displayed in the Pacman GUI.

        This evaluation function is meant for use with adversarial search agents
        (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
        This class provides some common elements to all of your
        multi-agent searchers.  Any methods defined here will be available
        to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

        You *do not* need to make any changes here, but you can if you want to
        add functionality to all your adversarial search agents.  Please do not
        remove anything, however.

        Note: this is an abstract class: one that should not be instantiated.  It's
        only partially specified, and designed to be extended.  Agent (game.py)
        is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
        Your minimax agent (question 2)
    """

    def getAction(self, gameState):
        """
            Returns the minimax action from the current gameState using self.depth
            and self.evaluationFunction.

            Here are some method calls that might be useful when implementing minimax.

            gameState.getLegalActions(agentIndex):
                Returns a list of legal actions for an agent
                agentIndex=0 means Pacman, ghosts are >= 1

            Directions.STOP:
                The stop direction, which is always legal

            gameState.generateSuccessor(agentIndex, action):
                Returns the successor game state after an agent takes an action

            gameState.getNumAgents():
                Returns the total number of agents in the game
        """
        "*** YOUR CODE HERE ***"
        
        PACMAN = 0
        FIRST_GHOST = 1
        LAST_GHOST = gameState.getNumAgents() - 1

        def max_terminal_test(state, depths_remaining, actions):
            return state.isLose() or not depths_remaining

        def min_terminal_test(state, depths_remaining):
            return state.isWin() or state.isLose()

        def initial_max_value(state, depths_remaining):
            actions = state.getLegalActions(PACMAN)
            if Directions.STOP in actions:
                actions.remove(Directions.STOP)
            if max_terminal_test(state, depths_remaining, actions):
                return self.evaluationFunction(state)
            successors = [(a, state.generateSuccessor(PACMAN, a)) for a in actions]
            return max(successors, key= lambda successor: min_value(successor[1], FIRST_GHOST, depths_remaining))[0]

        def max_value(state, depths_remaining):
            actions = state.getLegalActions(PACMAN)
            if Directions.STOP in actions:
                actions.remove(Directions.STOP)
            if max_terminal_test(state, depths_remaining, actions):
                return self.evaluationFunction(state)
            successors = [(a, state.generateSuccessor(PACMAN, a)) for a in actions]
            return max(min_value(s, FIRST_GHOST, depths_remaining) for a, s in successors)
        

        def min_value(state, current_ghost, depths_remaining):
            if min_terminal_test(state, depths_remaining):
                return self.evaluationFunction(state)
            actions = state.getLegalActions(current_ghost)
            successors = [(a, state.generateSuccessor(current_ghost, a)) for a in actions]

            if current_ghost == LAST_GHOST:
                return min(max_value(s, depths_remaining - 1) for a, s in successors)
            return min(min_value(s, current_ghost + 1, depths_remaining) for a, s in successors)
        

        return initial_max_value(gameState, self.depth)

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
        Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState):
        """
            Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"
        PACMAN = 0
        FIRST_GHOST = 1
        LAST_GHOST = gameState.getNumAgents() - 1

        def max_terminal_test(state, depths_remaining, actions):
            return state.isLose() or not depths_remaining

        def min_terminal_test(state, depths_remaining):
            return state.isWin() or state.isLose()


        def initial_max_value(state, depths_remaining):
            alfa = float('-inf')
            beta = float('inf')
            actions = state.getLegalActions(PACMAN)
            if Directions.STOP in actions:
                actions.remove(Directions.STOP)
            if max_terminal_test(state, depths_remaining, actions):
                return self.evaluationFunction(state)
            successors = [(a, state.generateSuccessor(PACMAN, a)) for a in actions]
            v = float('-inf')
            current_a = None
            for a, s in successors:
                new_v = min_value(s, FIRST_GHOST, depths_remaining, alfa, beta)
                if v < new_v:
                    v = new_v
                    current_a = a
                if v >= beta:
                    return current_a
                alfa = max(alfa, v)
            return current_a
        
            return max(successors, key= lambda successor: min_value(successor[1], FIRST_GHOST, depths_remaining, alfa, beta))[0]

        def max_value(state, depths_remaining, alfa, beta):
            actions = state.getLegalActions(PACMAN)
            if Directions.STOP in actions:
                actions.remove(Directions.STOP)
            if max_terminal_test(state, depths_remaining, actions):
                return self.evaluationFunction(state)
            successors = [(a, state.generateSuccessor(PACMAN, a)) for a in actions]
            v = float('-inf')
            for a, s in successors:
                v = max(v, min_value(s, FIRST_GHOST, depths_remaining, alfa, beta))
                if v >= beta:
                    return v
                alfa = max(alfa, v)
            return v
        

        def min_value(state, current_ghost, depths_remaining, alfa, beta):
            if min_terminal_test(state, depths_remaining):
                return self.evaluationFunction(state)
            actions = state.getLegalActions(current_ghost)
            successors = [(a, state.generateSuccessor(current_ghost, a)) for a in actions]

            v = float('inf')
            for a, s in successors:
                if current_ghost == LAST_GHOST:
                    v = min(v, max_value(s, depths_remaining - 1, alfa, beta))
                else: 
                    v = min(v, min_value(s, current_ghost + 1, depths_remaining, alfa, beta))
                if v <= alfa:
                    return v
                beta = min(beta, v)  
            return v     
        #print max_value(gameState, self.depth, float('-inf'), float('inf'))
        return initial_max_value(gameState, self.depth)
        util.raiseNotDefined()

class ExpectimaxAgent(MultiAgentSearchAgent):
    """
        Your expectimax agent (question 4)
    """

    def getAction(self, gameState):
        """
            Returns the expectimax action using self.depth and self.evaluationFunction

            All ghosts should be modeled as choosing uniformly at random from their
            legal moves.
        """
        "*** YOUR CODE HERE ***"
        PACMAN = 0
        FIRST_GHOST = 1
        LAST_GHOST = gameState.getNumAgents() - 1

        def max_terminal_test(state, depths_remaining, actions):
            return state.isLose() or not depths_remaining

        def min_terminal_test(state, depths_remaining):
            return state.isWin() or state.isLose()

        def initial_max_value(state, depths_remaining):
            actions = state.getLegalActions(PACMAN)
            if Directions.STOP in actions:
                actions.remove(Directions.STOP)
            if max_terminal_test(state, depths_remaining, actions):
                return Directions.STOP
                #self.evaluationFunction(state)
            successors = [(a, state.generateSuccessor(PACMAN, a)) for a in actions]
            return max(successors, key= lambda successor: min_value(successor[1], FIRST_GHOST, depths_remaining))[0]

        def max_value(state, depths_remaining):
            actions = state.getLegalActions(PACMAN)
            if Directions.STOP in actions:
                actions.remove(Directions.STOP)
            if max_terminal_test(state, depths_remaining, actions):
                return self.evaluationFunction(state)
            successors = [(a, state.generateSuccessor(PACMAN, a)) for a in actions]
            return max(min_value(s, FIRST_GHOST, depths_remaining) for a, s in successors)
        

        def min_value(state, current_ghost, depths_remaining):
            if min_terminal_test(state, depths_remaining):
                return self.evaluationFunction(state)
            actions = state.getLegalActions(current_ghost)
            successors = [(a, state.generateSuccessor(current_ghost, a)) for a in actions]

            if current_ghost == LAST_GHOST:
                expect_values = [max_value(s, depths_remaining - 1) for a, s in successors]
            else:
                expect_values = [min_value(s, current_ghost + 1, depths_remaining) for a, s in successors]
            
            return sum(expect_values)/len(expect_values)

        

        return initial_max_value(gameState, self.depth)
        util.raiseNotDefined()

def betterEvaluationFunction(currentGameState):
    """
        Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
        evaluation function (question 5).

        DESCRIPTION: <write something here so we know what you did>
        For this evaluation function, we used a weighted sum of the current score, the nearest food source (negative because being closer is better) and capsule(also negative) and the nearest ghost distance (positive because you want to be far from them).
        Besides that, we use also the nearest white ghost distance with a bigger weight than everthing else to incentivate the pacman to always try to eat them and increase the score 
    """
    "*** YOUR CODE HERE ***"
    #successorGameState = currentGameState.generatePacmanSuccessor(action)
    pos = currentGameState.getPacmanPosition()
    food = currentGameState.getFood()
    ghostStates = currentGameState.getGhostStates()
    scaredTimes = [ghostState.scaredTimer for ghostState in ghostStates]
    #import ipdb; ipdb.set_trace()
    capsules = currentGameState.getCapsules()
    capsule_distances = [manhattanDistance(capsule, pos) for capsule in capsules]
    nearest_capsule_distance = min(capsule_distances) if capsule_distances else 0

    ghost_distances = [manhattanDistance(ghostState.getPosition(), pos) for i, ghostState in enumerate(ghostStates) if not scaredTimes[i]]
    nearest_ghost_distance = min(ghost_distances) if ghost_distances else 0
    
    white_ghost_distances = [manhattanDistance(ghostState.getPosition(), pos) for i, ghostState in enumerate(ghostStates) if scaredTimes[i]]
    nearest_white_ghost_distance = min(ghost_distances) if ghost_distances else 0
    #delta_score = successorGameState.getScore() - currentGameState.getScore() 

    foods = food.asList()
    food_distances = [manhattanDistance(food, pos) for food in foods]
    nearest_food_distance = min(food_distances) if food_distances else 0

    #if currentGameState.getScore() < -300:
    #    import ipdb; ipdb.set_trace()
    #    return currentGameState.getScore() - nearest_food_distance - 2*nearest_white_ghost_distance
    
    #import ipdb; ipdb.set_trace()
    #if 8 > nearest_ghost_distance:# and nearest_capsule_distance < 6:
    #   return currentGameState.getScore() - nearest_food_distance - 2*nearest_white_ghost_distance - 2*nearest_capsule_distance + nearest_ghost_distance + 20
    nearest_food_distance = min(nearest_food_distance, nearest_capsule_distance)
    #for i, t in enumerate(scaredTimes):
    #    if t > ghost_distances[i]:
    #return currentGameState.getScore()  - nearest_food_distance + nearest_ghost_distance + 100#BONUS
    #print "aaaaaaaa"
    return currentGameState.getScore() - nearest_food_distance - nearest_capsule_distance - 3*nearest_white_ghost_distance + nearest_ghost_distance
    
    util.raiseNotDefined()


# Abbreviation
better = betterEvaluationFunction


class ContestAgent(AlphaBetaAgent):#MultiAgentSearchAgent):
    """
        Your agent for the mini-contest
    """

    def __init__(self, *args, **kworgs):
        MultiAgentSearchAgent.__init__(self, *args, **kworgs)
        self.x = 0
        self.start = -1
        self.vertices = None
        self.last = []
        self.lastg = {}
        self.lastg[1] = []
        self.lastg[2] = []
        self.lastg[3] = []
        self.TIMEOUT = 1.0

    def evalFn(self, currentGameState, previous=None):
        """
            Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
            evaluation function (question 5).

        """
        if currentGameState.isLose():
            return - 2000

        return 7000


    def evalState(self, state, previous):
        #if state.getScore() > 1730:
        #    import ipdb; ipdb.set_trace()
        score = 0
        score -= sum(len(state.getLegalActions(i)) for i in range(1, self.LAST_GHOST + 1))

        pos = state.getPacmanPosition()
        ghosts = state.getGhostStates()
        ghost_positions = [round_tuple(ghost.getPosition()) for ghost in ghosts if not ghost.scaredTimer]
        
        if state.isLose():
            score -= 10000

        white_ghosts = len([ghost for ghost in state.getGhostStates() if ghost.scaredTimer])
        white_ghosts_previous = len([ghost for ghost in previous.getGhostStates() if ghost.scaredTimer])

        capsules_count = len(state.getCapsules())
        capsules_count_previous = len(previous.getCapsules())
        
        nearest_ghost_distance = search(
            NearestColoredGhostProblem(pos, [], state)
        )[0]
        if nearest_ghost_distance != -1:
            score += nearest_ghost_distance

        if white_ghosts_previous > white_ghosts:
            score += 250*(white_ghosts_previous-white_ghosts)

        if white_ghosts and white_ghosts_previous:
            score -= 500*(capsules_count_previous - capsules_count)

        if white_ghosts:
            score += 150
            nearest_white_ghost = search(
                NearestWhiteGhostProblem(pos, ghost_positions+state.getCapsules(), state)
            )[0]
            if nearest_white_ghost != -1:
                score -= 5*nearest_white_ghost
                return score

        if capsules_count:
            nearest_capsule_distance = search(
                NearestCapsuleProblem(pos, ghost_positions, state)
            )[0]
            if nearest_capsule_distance != -1:
                score -= 5*nearest_capsule_distance
                return score

        nearest_food = search(
            NearestFoodProblem(pos, ghost_positions, state)
        )[0]
        if nearest_food != -1:
            score -= 5*nearest_food
            return score
        else:
            
            return score


    def is_trapped(self, state, depths_remaining, pacman_actions, n):
        if depths_remaining <= 0:
            return False
        if not pacman_actions:
            return False

        colored_ghost_positions = [round_tuple(ghost.getPosition()) for ghost in state.getGhostStates() if not ghost.scaredTimer]
        blocks = {}
        for i in range(n):
            blocks[i] = colored_ghost_positions
        trapped = search(NPacmanMovesProblem(state, n, blocks))
        if trapped or state.isLose():
            return True

        action = pacman_actions[0]
        successor = state.generateSuccessor(0, action)

        return self.min_is_trapped(successor, 1, depths_remaining, pacman_actions[1:], n)
    
    def min_is_trapped(self, state, current_ghost, depths_remaining, pacman_actions, n):
        colored_ghost_positions = [round_tuple(ghost.getPosition())  for ghost in state.getGhostStates() if not ghost.scaredTimer]
        blocks = {}
        for i in range(n):
            blocks[i] = colored_ghost_positions
        trapped = search(NPacmanMovesProblem(state, n, blocks))

        if state.isWin():
            return False
        if trapped or state.isLose():
            return True
        ghost_position = state.getGhostPosition(current_ghost)
        actions = state.getLegalActions(current_ghost)
        if ghost_position not in self.vertices:
            reverse = Actions.reverseDirection(state.getGhostState(current_ghost).getDirection())
            if reverse in actions:
                actions.remove(reverse)

        successors = [(a, state.generateSuccessor(current_ghost, a)) for a in actions]
        for a, s in successors:
            if current_ghost == state.getNumAgents() - 1:
                if self.is_trapped(s, depths_remaining - 1, pacman_actions, n):
                    return True
            elif self.min_is_trapped(state, current_ghost + 1, depths_remaining, pacman_actions, n):
                return True   

        return False

    

    def max_terminal_test(self, state, depths_remaining, actions):
        return state.isLose() or not depths_remaining or (time.time() - self.starttime)/self.actions > self.TIMEOUT

    def min_terminal_test(self, state, depths_remaining):
        return state.isWin() or state.isLose() or (time.time() - self.starttime)/self.actions > self.TIMEOUT

    def update_actions_pacman_near_capsule(self, state, actions):
        any_white = any(ghost.scaredTimer for ghost in state.getGhostStates())
        if any_white:
            return actions
        pos = state.getPacmanPosition()
        near_capsule = any(manhattanDistance(pos, capsule) == 1 for capsule in state.getCapsules())
        near_ghost = any(manhattanDistance(pos, ghost.getPosition()) < 3 for ghost in state.getGhostStates() if not ghost.scaredTimer)
        if near_capsule and not near_ghost:
            actions = [Directions.STOP]
        elif near_capsule and near_ghost:
            path = search(NearestCapsuleProblem(pos, [], state))
            actions = [path[1][0]]
        return actions

    def initial_max_value(self, state, depths_remaining):
        alfa = float('-inf')
        beta = float('inf')
        actions = state.getLegalActions(self.PACMAN)
        if Directions.STOP in actions:
            actions.remove(Directions.STOP)
        if self.max_terminal_test(state, depths_remaining, actions):
            return Directions.STOP
        #import ipdb; ipdb.set_trace()

        actions = self.update_actions_pacman_near_capsule(state, actions)
        successors = [(a, state.generateSuccessor(self.PACMAN, a)) for a in actions]
        v = float('-inf')
        current_a = None
        alis = []
        if (all(manhattanDistance(state.getPacmanPosition(), ghost.getPosition()) > depths_remaining for ghost in state.getGhostStates() if not ghost.scaredTimer)):
            alis = successors
            successors = []
        
        for a, s in successors:

            new_v = self.min_value(s, self.FIRST_GHOST, depths_remaining, alfa, beta, state)
            if v < new_v:
                v = new_v
                current_a = a
                alis = [(a, s)]
            elif v == new_v:
                alis.append((a, s))
            alfa = max(alfa, v)
        #import ipdb; ipdb.set_trace()
        if v == -2000:
            print "trapped"
            #import ipdb;ipdb.set_trace()
            print state
        return alis 
        return max(alis, key=lambda act: self.evalState(act[1], state))[0]
        #return current_a
        
        #return max(successors, key= lambda successor: min_value(successor[1], FIRST_GHOST, depths_remaining, alfa, beta))[0]

    def max_value(self, state, depths_remaining, alfa, beta, previous):
        actions = state.getLegalActions(self.PACMAN)
        if Directions.STOP in actions:
            actions.remove(Directions.STOP)
        if self.max_terminal_test(state, depths_remaining, actions):
            return self.evaluationFunction(state, previous)
        
        actions = self.update_actions_pacman_near_capsule(state, actions)
        successors = [(a, state.generateSuccessor(self.PACMAN, a)) for a in actions]
        if (all(manhattanDistance(state.getPacmanPosition(), ghost.getPosition()) > depths_remaining for ghost in state.getGhostStates() if not ghost.scaredTimer)):
            return 7000

        
        v = float('-inf')
        for a, s in successors:
            v = max(v, self.min_value(s, self.FIRST_GHOST, depths_remaining, alfa, beta, state))
            if v >= beta:
                return v
            alfa = max(alfa, v)
        return v
    

    def min_value(self, state, current_ghost, depths_remaining, alfa, beta, previous):
        if manhattanDistance(state.getGhostPosition(current_ghost), state.getPacmanPosition()) >= depths_remaining:
            #print "far", manhattanDistance(state.getGhostPosition(current_ghost), state.getPacmanPosition())
            if current_ghost == self.LAST_GHOST:
                return self.max_value(state, depths_remaining - 1, alfa, beta, previous)
            else: 
                return self.min_value(state, current_ghost + 1, depths_remaining, alfa, beta, previous)

        if self.min_terminal_test(state, depths_remaining):
            return self.evaluationFunction(state, previous)
        ghost_position = state.getGhostPosition(current_ghost)
        actions = state.getLegalActions(current_ghost)
        if ghost_position not in self.vertices:
            reverse = Actions.reverseDirection(state.getGhostState(current_ghost).getDirection())
            if reverse in actions:
                actions.remove(reverse)
        #if len(actions) >= 3:
        #    depths_remaining = max(0, min(6, depths_remaining - 4))

        successors = [(a, state.generateSuccessor(current_ghost, a)) for a in actions]

        v = float('inf')
        for a, s in successors:
            if current_ghost == self.LAST_GHOST:
                v = min(v, self.max_value(s, depths_remaining - 1, alfa, beta, previous))
            else: 
                v = min(v, self.min_value(s, current_ghost + 1, depths_remaining, alfa, beta, previous))
            if v <= alfa:
                return v
            beta = min(beta, v)  
        return v     
    

    def getAction(self, gameState, bs=[]):
        """
            Returns an action.  You can use any method you want and search to any depth you want.
            Just remember that the mini-contest is timed, so you have to trade off speed and computation.

            Ghosts don't behave randomly anymore, but they aren't perfect either -- they'll usually
            just make a beeline straight towards Pacman (or away from him if they're scared!)
        """

        if not self.vertices:
            walls = gameState.getWalls()
            self.paths_list, self.vertices, fat_paths = grid_to_graph(gameState, invert_grid(walls))
            self.groups, self.paths, self.vertex_paths, self.edges = make_edges(gameState, self.paths_list, self.vertices)
            self.PACMAN = 0
            self.FIRST_GHOST = 1
            self.LAST_GHOST = gameState.getNumAgents() - 1
        #import ipdb; ipdb.set_trace()
        """  
        import __main__
        if '_display' in dir(__main__):
            if 'drawExpandedCells' in dir(__main__._display): #@UndefinedVariable
                __main__._display.drawExpandedCells(self.groups[gameState.getScore() % len(self.groups)]) #@UndefinedVariable
        """
        self.evaluationFunction = self.evalFn
        self.depth = 7


        if gameState.getScore() == 0:
            self.start = 0;
            self.starttime = time.time()
            self.actions = 0

        self.actions += 1

        #self.lastg[1].append(gameState.getGhostPosition(1))
        #self.lastg[2].append(gameState.getGhostPosition(2))
        #self.lastg[3].append(gameState.getGhostPosition(3))
        
        #self.last.append(gameState.getPacmanPosition())
        #if len(self.last) > self.depth:
        #    self.last = self.last[1:self.depth+1]
        #    self.lastg[1] = self.lastg[1][1:self.depth+1]
        #    self.lastg[2] = self.lastg[2][1:self.depth+1]
        #    self.lastg[3] = self.lastg[3][1:self.depth+1]
        #import __main__
        #if '_display' in dir(__main__):
        #    if 'drawExpandedCells' in dir(__main__._display): #@UndefinedVariable
        #        self.display = __main__._display
        #        #import ipdb; ipdb.set_trace()
        #self.display.drawExpandedCells(self.last + self.lastg[1] + self.lastg[2] + self.lastg[3]) #@UndefinedVariable
            
        if self.start < 4:
            self.start += 1
            return [Directions.EAST, Directions.EAST, Directions.NORTH, Directions.NORTH, Directions.NORTH][self.start - 1]

        currenttime = time.time() - self.starttime
        if self.actions < 290:
            self.TIMEOUT = (180 - currenttime)/(290 - self.actions)
        if currenttime > 150:
            self.TIMEOUT = 0.3
        time_per_action = currenttime / self.actions

        #print time_per_action, self.actions, self.TIMEOUT

        #if time_per_action < self.TIMEOUT:
        alis = self.initial_max_value(gameState, self.depth)
        actions = [x[0] for x in alis]
        #if gameState.generateSuccessor(0, action).isLose():
            
        #return action
        #print "old action"
        #import ipdb; ipdb.set_trace()
        
        depth = 0
        n = 1
        check_actions = 3
        ghost_moves = 1
        ghosts = [search(GhostMovesProblem(round_tuple(ghost.getPosition()),ghost.scaredTimer, gameState, n)) for ghost in gameState.getGhostStates()]
        # remove blocks later
        #blocks = {}
        #for i in range(n):
        #    blocks[i] = []
        #    blocks[i].extend(bs)
        #    for gn in ghosts:
        #        blocks[i].extend([round_tuple(t) for t in gn[i]])
        pacman = gameState.getPacmanPosition()
        
        ghostStates = gameState.getGhostStates()
        capsules = gameState.getCapsules()
        colored_ghost_positions = [ghost.getPosition() for ghost in gameState.getGhostStates() if not ghost.scaredTimer]
        white_ghost_positions = [(ghost.getPosition(), ghost.scaredTimer) for ghost in gameState.getGhostStates() if ghost.scaredTimer]
        if white_ghost_positions:
            paths_to_white_ghosts = [search(AStartMazeSearchProblem(pacman, ghost[0], colored_ghost_positions+capsules, gameState)) for ghost in white_ghost_positions]
            valids = filter((lambda x: x[0] != -1), paths_to_white_ghosts)

            while valids:
                m = min(valids, key=lambda x:x[0])
                pos = Actions.getSuccessor(pacman, m[1][0])
                if m[1][0] in actions:
                #if not self.is_trapped(gameState, depth, m[1], n) and not (pos == (9,5) or pos == (10, 5) or pos == (8, 5)):
                    return m[1][0]
                valids.remove(m)

        elif capsules:
            near_ghost = any(manhattanDistance(pacman, ghost.getPosition()) < 3 for ghost in gameState.getGhostStates() if not ghost.scaredTimer)
        
            paths_to_capsules = [search(AStartMazeSearchProblem(pacman, capsule, colored_ghost_positions, gameState)) for capsule in capsules]
            valids = filter((lambda x: x[0] != -1), paths_to_capsules)
            
            while valids:
                m = min(valids, key=lambda x:x[0])
                if m[0] == 1 and not near_ghost:
                    return Directions.STOP
                if m[1][0] in actions:
                #if not self.is_trapped(gameState, depth, m[1], n):
                    return m[1][0]
                valids.remove(m)


        nfood = search(NearestFoodProblem(pacman, colored_ghost_positions, gameState))
        if nfood[0] != -1:
            action = nfood[1][0]
            if action in actions:
            #if not self.is_trapped(gameState, depth, nfood[1], n):
                return action


        #trapped = search(NPacmanMovesProblem(gameState, n, blocks))
        #if trapped:
        #    print "trapped ", pacman 
        #alt_depth = self.depth
        #self.depth = 4
        #action = AlphaBetaAgent.getAction(self, gameState)
        #self.depth = alt_depth
        #suc = gameState.generateSuccessor(0, action)
        #if suc.isLose():
        #    print "Lose: ", pacman
        return max(alis, key=lambda act: self.evalState(act[1], gameState))[0]
