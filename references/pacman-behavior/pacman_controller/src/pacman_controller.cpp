#include "ros/ros.h"
#include "pacman_interface/PacmanAction.h"
#include "std_msgs/String.h"

#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <string.h>

#include "pacman_controller/game_info.h"
#include "keyboard_agent.h"
#include "visible_ghost_agent.h"

#define NUMER_OF_GHOSTS 4

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pacman_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(60);

    GameInfo gameInfo;

    KeyboardAgent *pacmanAgent = new KeyboardAgent;
    vector< VisibleGhostAgent* > ghosts;
    for(int i = 0 ; i < NUMER_OF_GHOSTS ; i++)
        ghosts.push_back(new VisibleGhostAgent);

    for(int i = 0 ; i < NUMER_OF_GHOSTS ; i++)
        ROS_INFO_STREAM(i << " - " << ghosts[i]->getAgentName() << " - " << ghosts[i]->getGhostNumber());

    while (ros::ok())
    {
        pacmanAgent->sendAction();

        ros::spinOnce();
        loop_rate.sleep();
    }
}