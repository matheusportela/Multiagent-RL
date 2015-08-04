#!/usr/bin/env python

# keyboardAgents.py
# -----------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

import rospy
from pacman_interface.msg import PacmanAction
from std_msgs.msg import String

class RosKeyboardAgent():
    """
    An agent controlled by ROS messages.
    """
    # NOTE: Arrow keys also work.
    WEST_KEY  = 'a'
    EAST_KEY  = 'd'
    NORTH_KEY = 'w'
    SOUTH_KEY = 's'
    STOP_KEY = 'q'

    keys_pressed = []
    keys = []

    def __init__(self):
        pub = rospy.Publisher('/pacman_interface/pacman_action', PacmanAction, queue_size=10)
        rospy.Subscriber("/pacman_interface/keypress", String, self.keypressCallback)

        r = rospy.Rate(60) # 10hz
        while not rospy.is_shutdown():
            move = self.getAction()
            if(move != None):
                pacmanAction = PacmanAction()
                pacmanAction.action = move
                pub.publish(pacmanAction)
            r.sleep()

    def keypressCallback(self, data):
        self.keys_pressed += [data.data]
    def keys_waiting(self):
        if self.keys_pressed != []:
            self.keys = self.keys_pressed
            self.keys_pressed = []
        return self.keys

    def getAction(self):
        keys = self.keys_waiting()
        move = self.getMove(keys)
        return move

    def getMove(self, keys):
        move = None
        if   (self.STOP_KEY in keys) : move = PacmanAction.STOP
        if   (self.WEST_KEY in keys or 'Left' in keys) : move = PacmanAction.WEST
        if   (self.EAST_KEY in keys or 'Right' in keys): move = PacmanAction.EAST
        if   (self.NORTH_KEY in keys or 'Up' in keys):   move = PacmanAction.NORTH
        if   (self.SOUTH_KEY in keys or 'Down' in keys): move = PacmanAction.SOUTH
        return move

def runPacmanKeyboard():
    rospy.init_node('pacman_keyboard_controller', anonymous=True)
    RosKeyboardAgent()

if __name__ == '__main__':
    try:
        runPacmanKeyboard()
    except rospy.ROSInterruptException: pass

