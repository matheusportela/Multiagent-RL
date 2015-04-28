# pacmanAgents.py
# ---------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


from pacman import Directions
from game import Agent
import random
import game
import util

import learn

class LeftTurnAgent(game.Agent):
    "An agent that turns left at every opportunity"

    def getAction(self, state):
        legal = state.getLegalPacmanActions()
        current = state.getPacmanState().configuration.direction
        if current == Directions.STOP: current = Directions.NORTH
        left = Directions.LEFT[current]
        if left in legal: return left
        if current in legal: return current
        if Directions.RIGHT[current] in legal: return Directions.RIGHT[current]
        if Directions.LEFT[left] in legal: return Directions.LEFT[left]
        return Directions.STOP

class GreedyAgent(Agent):
    def __init__(self, evalFn="scoreEvaluation"):
        self.evaluationFunction = util.lookup(evalFn, globals())
        assert self.evaluationFunction != None

    def getAction(self, state):
        # Generate candidate actions
        legal = state.getLegalPacmanActions()
        if Directions.STOP in legal: legal.remove(Directions.STOP)

        successors = [(state.generateSuccessor(0, action), action) for action in legal]
        scored = [(self.evaluationFunction(state), action) for state, action in successors]
        bestScore = max(scored)[0]
        bestActions = [pair[1] for pair in scored if pair[0] == bestScore]
        return random.choice(bestActions)

class RandomAgent(game.Agent):
    def getAction(self, state):
        actions = state.getLegalPacmanActions()
        return random.choice(actions)


class QLearnAgent(game.Agent):
    """Agent with Q-learning algorithm."""

    def __init__(self, evalFn='scoreEvaluation'):
        self.adapter = None
        self.previous_state = None
        self.previous_action = 'Stop'

    def initializeAdapter(self, state):
        """Initialize system adapter to use the learning algorithm."""
        width = state.data.layout.width
        height = state.data.layout.height
        self.adapter = learn.PacmanSystemAdapter(width, height)

    def convertStateToMeasurements(self, state):
        """Convert game state to measurement understandable by the learning
        algorithm.
        """
        pacman_position = state.getPacmanState().configuration.pos
        ghosts_positions = [ghost.configuration.pos for ghost in state.getGhostStates()]
        reward = state.getScore() - self.previous_state.getScore() if self.previous_state else 0
        measurements = learn.PacmanMeasurements(
            pacman_position=pacman_position,
            ghosts_positions=ghosts_positions,
            action=self.previous_action,
            reward=reward,
        )

        return measurements

    def getAction(self, state):
        """Execute learning algorithm and select next action to be executed by
        the Pacman.
        """
        if not self.adapter:
            self.initializeAdapter(state)

        measurements = self.convertStateToMeasurements(state)
        action = self.adapter.run(measurements)

        print measurements

        if action not in state.getLegalPacmanActions():
            action = 'Stop'

        self.previous_state = state
        self.previous_action = action
        return action


def scoreEvaluation(state):
    return state.getScore()
