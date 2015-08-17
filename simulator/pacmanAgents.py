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

import pacman_learn

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


class InterfaceAgent(game.Agent):
    def __init__(self):
        self.actions = ['North', 'South', 'East', 'West', 'Stop']
        self.learn_interface = None

    def get_state_dimensions(self, state):
        # Layout includes the walls, hence, the sizes are larger by 2 when
        # considering only allowed places
        width = state.data.layout.width - 2
        height = state.data.layout.height - 2
        size = width*height
        return width, height, size

    def calculate_agent_position_index(self, state, agent_index):
        width, height, size = self.get_state_dimensions(state)

        if (agent_index == 0):
            agent_position = state.getPacmanPosition()
        else:
            agent_position = state.getGhostPosition(agent_index)

        # Offset to get position in the interval [0, width] and [0, height]
        x = agent_position[0] - 1
        y = agent_position[1] - 1
        index = agent_index*size + y*width + x

        return index

    def calculate_state_index(self, state):
        state_index = 0
        
        for agent_index in xrange(state.getNumAgents()):
            state_index += self.calculate_agent_position_index(state, agent_index)

        return state_index

    def calculate_num_states(self, state):
        num_agents = state.getNumAgents()
        _, _, size = self.get_state_dimensions(state)
        return size*num_agents


    def calculate_action_index(self, action):
        return self.actions.index(action)


    def getAction(self, state):
        if not self.learn_interface:
            initial_state = self.calculate_state_index(state)
            num_actions = len(self.actions)
            num_states = self.calculate_num_states(state)
            self.learn_interface = pacman_learn.PacmanProblemAdapter(
                initial_state=initial_state, num_actions=num_actions,
                num_states=num_states)

        print 'Getting action'
        actions = state.getLegalPacmanActions()
        action = random.choice(actions)
        print 'Action: ', self.calculate_action_index(action)
        print 'State:', self.calculate_state_index(state)
        raw_input()
        return 'East'



def scoreEvaluation(state):
    return state.getScore()
