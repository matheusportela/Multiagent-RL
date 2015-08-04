#include "ros/ros.h"

#include "pacman_msgs/PacmanAction.h"
#include "pacman_msgs/StartGame.h"

#include <mcheck.h>

int NUMBER_OF_GAMES = 10005;
int NUMBER_OF_TRAININGS = 5005;
bool is_training = true;

int main(int argc, char **argv)
{
    // start ros
    ros::init(argc, argv, "q_learning");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    srand (time(NULL)); // start random fucntions

    ros::Publisher chatter_pub = n.advertise<pacman_msgs::PacmanAction>("/random_topic", 1000);

    // client to start game service and server for end game service
    ros::ServiceClient start_game_client = n.serviceClient<pacman_msgs::StartGame>("/pacman/start_game");
    ros::service::waitForService("/pacman/start_game", -1);

    // spin to answer services
    ros::spin();

    // shutdown ros node
    ros::shutdown();
}

// Working for deterministic non behavioral games