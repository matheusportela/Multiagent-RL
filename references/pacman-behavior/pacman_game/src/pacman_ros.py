#!/usr/bin/env python

import rospy
import pacman
from pacman_msgs.srv import StartGame
from pacman_msgs.srv import EndGame

import gc


start_game = False
show_gui = False


class PacmanGame():
    """
    A game started and controlled by ROS messages and services.
    """
    def __init__( self, index = 0 ):
        #start ros node
        rospy.init_node('pacman_game', anonymous=True)
        self.r = rospy.Rate(10) # 10hz

        # game attributes
        # game_attributes = ["-p", "RosWaitServiceAgent", "-l", "smallClassic", "-k", "4"]
        # game_attributes = ["-p", "RosServiceWithErrorsAgent", "-l", "smallClassic", "-k", "4"]
        game_attributes = ["-p", "RosServiceWithErrorsAgent", "-l", "originalClassic", "-k", "4"]
        #a=["-p", "RosWaitServiceAgent", "-l", "originalClassic", "-k", "4"]
        #a=["-p", "RosWaitServiceAgent", "-l", "mediumGrid", "-k", "4"]
        self.args = pacman.readCommand(game_attributes)

        self.args['send_pose_as_service'] = True
        self.args['send_pose_with_error'] = True
        self.args['ghost_distance_error'] = 0.01
        self.args['pacman_pose_error'] = 0.01

        # service and variables to start new game and end it
        self.start_game_srv = rospy.Service('/pacman/start_game', StartGame, self.start_game_service)
        self.end_game_client = rospy.ServiceProxy('/pacman/end_game', EndGame)
        self.start_game = False
        self.show_gui = False

        self.game_counter = 0

    def start_game_service(self, req):
        if self.start_game:
            rospy.logwarn("Trying to start already started game")
            return False
        self.start_game = True
        self.show_gui = req.show_gui
        return True

    def end_game(self, is_win, match_score):
        # set game as not running
        self.start_game = False
        self.show_gui = False

        # call end game service
        ending_game_string = "Ending game " + str(self.game_counter)
        rospy.loginfo(ending_game_string)
        rospy.wait_for_service('/pacman/end_game')
        try:
          srv_resp = self.end_game_client(is_win, match_score)
          if not srv_resp.game_restarted:
            rospy.signal_shutdown("Shuting down node, game ended and wasn't restarted")
        except rospy.ServiceException as exc:
          print("Service did not process request: " + str(exc))

        return True

    def runSingleGame(self):

        # run game and get if win or lose
        self.args['pacman'].startEpisode()
        games = pacman.runGames(**self.args)
        single_game = games.pop()
        is_win = single_game.state.isWin()
        match_score = single_game.state.getScore()

        # end game
        self.end_game(is_win, match_score)


    # run game
    def run(self):
        #gc.set_debug(gc.DEBUG_LEAK)
        gc.enable()

        while not rospy.is_shutdown():
            if self.start_game:
                self.game_counter += 1

                #if not show gui, set game as in training mode
                if not self.show_gui:
                    self.args['numTraining'] = 1
                else:
                    self.args['numTraining'] = 0

                self.runSingleGame()

            gc.collect()
            len(gc.get_objects())

            #sleep while in loop
            self.r.sleep()

if __name__ == '__main__':
    game = PacmanGame()
    try:
        game.run()
    except rospy.ROSInterruptException: pass
    
