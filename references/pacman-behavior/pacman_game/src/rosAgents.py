# keyboardAgents.py
# -----------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

from game import Agent
from game import Directions
import random
import rospy
from pacman_msgs.msg import PacmanAction
from pacman_msgs.srv import PacmanGetAction
from pacman_msgs.srv import RewardService

class LearningAgent(Agent):

    def __init__(self):
        """
        initialize the learning agent
        """
        print "Starting agent"
        self.rosGiveReward = rospy.ServiceProxy('/pacman/reward', RewardService)

    def startEpisode(self):
        self.lastState = None

    def doAction(self,state,action):
        """
            Called by inherited class when
            an action is taken in a state
        """
        self.lastState = state
        self.lastAction = action
        
    def observationFunction(self, state):
        """
            This is where we ended up after our last action.
            The simulation should somehow ensure this is called
        """
        if not self.lastState is None:
            reward = state.getScore() - self.lastState.getScore()
            self.observeTransition(self.lastState, self.lastAction, state, reward)
        return state

    def observeTransition(self, state,action,nextState,deltaReward):
        """
            Called by environment to inform agent that a transition has
            been observed. This will result in a call to self.update
            on the same arguments

            NOTE: Do *not* override or call this function
        """

        # give reward
        # TODO: check if ok to comment this
        # rospy.wait_for_service('/pacman/reward')
        # service called here
        try:
            self.rosGiveReward(deltaReward)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def final(self, state):
        """
          Called by Pacman game at the terminal state
        """
        #print "Ending game"
        #print "Score ", state.getScore()
        deltaReward = state.getScore() - self.lastState.getScore()
        self.observeTransition(self.lastState, self.lastAction, state, deltaReward)

class RosAgent(Agent):
    """
    An agent controlled by ROS messages.
    """
    directions = [Directions.WEST,
                  Directions.EAST,
                  Directions.NORTH,
                  Directions.SOUTH,
                  Directions.STOP]

    def __init__( self, index = 0 ):

        self.nextMove = None
        self.lastMove = Directions.STOP
        self.index = index
        self.keys = []
        
        rospy.Subscriber("/pacman/pacman_action", PacmanAction, self.actionCallback)

    def actionCallback(self, data):
        self.nextMove = None

        if data.action >= 0 and data.action <= 5:
            self.nextMove = self.directions[data.action]

    def getAction(self, state):
       # from graphicsUtils import keys_waiting
       # from graphicsUtils import keys_pressed
       # keys = keys_waiting() + keys_pressed()
       # if keys != []:
       #     self.keys = keys

        legal = state.getLegalActions(self.index)
        move = self.getMove(legal)

        if move == None:
            # Try to move in the same direction as before if impossible, stop
            if self.lastMove in legal:
                move = self.lastMove
            else:
                move = Directions.STOP

        if move not in legal:
            move = random.choice(legal)

        self.lastMove = move
        return move

    def getMove(self, legal):
        move = None

        if self.nextMove in legal:
            move = self.nextMove
        self.nextMove = None

        return move


class RosWaitServiceAgent(LearningAgent):
    """
    An agent controlled by ROS messages.
    """
    directions = [Directions.WEST,
                  Directions.EAST,
                  Directions.NORTH,
                  Directions.SOUTH,
                  Directions.STOP]
    actionToMovement = {PacmanAction.WEST:  Directions.WEST,
                        PacmanAction.EAST:  Directions.EAST,
                        PacmanAction.NORTH: Directions.NORTH,
                        PacmanAction.SOUTH: Directions.SOUTH,
                        PacmanAction.STOP:  Directions.STOP}

    def __init__( self, index = 0 ):
        super(RosWaitServiceAgent, self).__init__()

        self.nextMove = None
        self.lastMove = Directions.STOP
        self.index = index
        self.keys = []

        self.rosGetAction = rospy.ServiceProxy('/pacman/get_action', PacmanGetAction)


    def getAction(self, state):
        legal = state.getLegalActions(self.index)
        move = Directions.STOP

        try:
            servResponse = self.rosGetAction()
            move = self.actionToMovement[servResponse.action]
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        if move not in legal:
            move = Directions.STOP

        self.doAction(state, move)
        return move

    def fromActionToMove(self, action):
        move = None

        return move


class RosServiceWithErrorsAgent(LearningAgent):
    """
    An agent controlled by ROS messages.
    """
    directions = [Directions.WEST,
                  Directions.EAST,
                  Directions.NORTH,
                  Directions.SOUTH,
                  Directions.STOP]
    actionToMovement = {PacmanAction.WEST:  Directions.WEST,
                        PacmanAction.EAST:  Directions.EAST,
                        PacmanAction.NORTH: Directions.NORTH,
                        PacmanAction.SOUTH: Directions.SOUTH,
                        PacmanAction.STOP:  Directions.STOP}
    chance_of_move_error = 0.001

    def __init__( self, index = 0 ):
        super(RosServiceWithErrorsAgent, self).__init__()

        self.nextMove = None
        self.lastMove = Directions.STOP
        self.index = index
        self.keys = []

        self.rosGetAction = rospy.ServiceProxy('/pacman/get_action', PacmanGetAction)


    def getAction(self, state):
        legal = state.getLegalActions(self.index)
        move = Directions.STOP

        try:
            servResponse = self.rosGetAction()
            move = self.actionToMovement[servResponse.action]
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        is_error = random.random()
        if is_error < self.chance_of_move_error:
            error_directions = list(self.directions)
            error_directions.remove(move)
            move = random.choice(error_directions)

        if move not in legal:
            move = Directions.STOP

        self.doAction(state, move)
        return move

    def fromActionToMove(self, action):
        move = None

        return move



class RosWaitAgent(Agent):
    """
    An agent controlled by ROS messages.
    """
    directions = [Directions.WEST,
                  Directions.EAST,
                  Directions.NORTH,
                  Directions.SOUTH,
                  Directions.STOP]

    def __init__( self, index = 0 ):

        self.nextMove = None
        self.index = index
        self.keys = []
        self.r = rospy.Rate(10)
        rospy.Subscriber("/pacman/pacman_action", PacmanAction, self.actionCallback)

    def actionCallback(self, data):
        self.nextMove = None

        if data.action >= 0 and data.action <= 5:
            self.nextMove = self.directions[data.action]
        else:
            rospy.logwarn("Inexistent action received!")

    def getAction(self, state):
       # from graphicsUtils import keys_waiting
       # from graphicsUtils import keys_pressed
       # keys = keys_waiting() + keys_pressed()
       # if keys != []:
       #     self.keys = keys

        legal = state.getLegalActions(self.index)

        move = self.getMove(legal)

        while move == None:
            move = self.getMove(legal)
            self.r.sleep()
            if rospy.is_shutdown():
                exit()

        if move not in legal:
            rospy.logerr("Illegal move received, executing random action!")
            move = random.choice(legal)

        return move

    def getMove(self, legal):
        move = None

        if self.nextMove in legal:
            move = self.nextMove
        else:
            if self.nextMove != None:
                rospy.logerr("Illegal action " + str(self.nextMove) + " received! Legal: " + str(legal))
                move = Directions.STOP
        self.nextMove = None

        return move