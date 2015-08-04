#!/usr/bin/env python

# multiAgents.py
# --------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

import rospy
import rospkg
import sys
from pacman_interface.msg import PacmanAction
from pacman_interface.msg import AgentAction
from pacman_interface.srv import PacmanGetAction
from pacman_interface.srv import PacmanInitializationInfo

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('pacman_interface') + '/src'

sys.path.append(pkg_path)

from copy import copy
from game import Directions
from game import Actions
from game import Agent
import time
from mypy import NPacmanMovesProblem, AStartMazeSearchProblem, GhostMovesProblem, NearestFoodProblem, search, round_tuple
from mypy import grid_to_graph, invert_grid, make_edges, NearestCapsuleProblem
from mypy import NearestWhiteGhostProblem, NearestColoredGhostProblem, nearest_distances

from pacman import GameState
import layout

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
    
    moveToAction = {   Directions.WEST:  PacmanAction.WEST,
                       Directions.EAST:  PacmanAction.EAST,
                       Directions.NORTH: PacmanAction.NORTH,
                       Directions.SOUTH: PacmanAction.SOUTH,
                       Directions.STOP:  PacmanAction.STOP,}
    actionToMovement = {PacmanAction.WEST:  Directions.WEST,
                        PacmanAction.EAST:  Directions.EAST,
                        PacmanAction.NORTH: Directions.NORTH,
                        PacmanAction.SOUTH: Directions.SOUTH,
                        PacmanAction.STOP:  Directions.STOP}

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = scoreEvaluationFunction
        self.depth = int(depth)
        self.gameState = GameState()

        self.initializeGameState()

        rospy.Subscriber("/pacman_interface/agent_action", AgentAction, self.agentActionCallback)
        rospy.Service('get_action', PacmanGetAction, self.getAction)

        rospy.spin()

    def initializeGameState(self):
        rospy.wait_for_service('pacman_inialize_game_state')
        try:
            getInitializationInfo = rospy.ServiceProxy('pacman_inialize_game_state', PacmanInitializationInfo)
            initInfo = getInitializationInfo()

            thisLayout = layout.getLayout(initInfo.layout)
            numGhosts = initInfo.numGhosts

            self.gameState.initialize(thisLayout, numGhosts)
            print "Game initialized"
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def agentActionCallback(self, data):
        agentIndex = data.agentIndex
        move = self.actionToMovement[data.action]
        self.gameState = self.gameState.generateSuccessor(agentIndex, move)

class ExpectimaxAgent(MultiAgentSearchAgent):
    """
        Your expectimax agent
    """
    def getAction(self, request):
        """
            Returns the expectimax action using self.depth and self.evaluationFunction

            All ghosts should be modeled as choosing uniformly at random from their
            legal moves.
        """
        gameState = self.gameState
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
        
        move = initial_max_value(gameState, self.depth)
        action = PacmanAction()
        action.action = self.moveToAction[move]
        
        
        
        return action

def runPacmanMinMax():
    rospy.init_node('pacman_expectimax', anonymous=True)
    ExpectimaxAgent()

if __name__ == '__main__':
    try:
        runPacmanMinMax()
    except rospy.ROSInterruptException: pass

