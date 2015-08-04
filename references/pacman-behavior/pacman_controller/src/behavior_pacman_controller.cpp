#include "ros/ros.h"
#include "pacman_interface/PacmanAction.h"

#include "pacman_controller/game_info.h"

#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <string.h>

#include "behavior_keyboard_agent.h"
#include "visible_ghost_agent.h"

#define NUMER_OF_GHOSTS 4

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pacman_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    GameInfo game_info;
    game_info.printMap();
    game_info.precalculateAllDistances();

    BehaviorKeyboardAgent *pacmanAgent = new BehaviorKeyboardAgent();

    while (ros::ok())
    {
        pacmanAgent->sendAction(game_info);

        ros::spinOnce();
        loop_rate.sleep();
    }
}