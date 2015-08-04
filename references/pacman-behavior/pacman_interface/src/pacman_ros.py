#!/usr/bin/env python

import rospy
import pacman

def runPacman():
    rospy.init_node('pacman_interface', anonymous=True)

    a=["-p", "RosWaitAgent", "-l", "originalClassic", "-k", "4"]
    args = pacman.readCommand(a)
    pacman.runGames(**args)


if __name__ == '__main__':
    try:
        runPacman()
    except rospy.ROSInterruptException: pass
    
