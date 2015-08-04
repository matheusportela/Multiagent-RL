#include "ros/ros.h"

#include "pacman_msgs/PacmanAction.h"
#include "pacman_msgs/StartGame.h"
#include "pacman_msgs/EndGame.h"
#include "pacman_msgs/PacmanGetAction.h"
#include "pacman_msgs/RewardService.h"

#include "q_learning_pacman/bayesian_game_state.h"
#include "q_learning_pacman/behavior_agent.h"
#include "q_learning_pacman/q_learning.h"

int NUMBER_OF_GAMES = 7;
int NUMBER_OF_TRAININGS = 0;

bool endGame(pacman_msgs::EndGame::Request &req, pacman_msgs::EndGame::Response &res, 
        ros::ServiceClient *start_game_client, BayesianGameState **game_state)
{
    // count number of games
    static int game_count = 0;
    game_count++;

    if (req.win)
    {
        ROS_INFO_STREAM("Won game " << game_count);
    }
    else
    {
        ROS_WARN_STREAM("Lost game " << game_count);
    }

    if (game_count < NUMBER_OF_GAMES)
    {
        pacman_msgs::StartGame start_game;

        if (game_count < NUMBER_OF_TRAININGS)
            start_game.request.show_gui = false;
        else
            start_game.request.show_gui = true;

        if (start_game_client->call(start_game))
            if(start_game.response.started)
            {
                // new game started
                delete *game_state;
                *game_state = new BayesianGameState();
                res.game_restarted = true;

                ROS_INFO("New game started");
                return true;
            }
            else
                ROS_ERROR("Failed to start game (check if game already started)");
        else // if problem => print error
            ROS_ERROR("Failed to call service StartGame");
    }

    // game not restarted
    res.game_restarted = false;
    return true;
}

bool getAction(pacman_msgs::PacmanGetAction::Request &req, pacman_msgs::PacmanGetAction::Response &res, 
                    BayesianGameState **game_state, BehaviorAgent pacman, QLearning *q_learning)
{
    ROS_INFO_STREAM("Sending action");

    // predict next game state
    int behavior = q_learning->getBehavior(*game_state);
    //(*game_state)->printPacmanOrGhostPose(false, 0);
    //(*game_state)->printPacmanOrGhostPose(false, 1);
    //(*game_state)->printPacmanOrGhostPose(true, 0);
    pacman_msgs::PacmanAction action = pacman.getAction(*game_state, 1);
    (*game_state)->predictAgentsMoves(action);

    // game not restarted
    res.action = action.action;
    return true;
}

bool receiveReward(pacman_msgs::RewardService::Request &req, pacman_msgs::RewardService::Response &res, 
                    BayesianGameState **game_state, QLearning *q_learning)
{
    int reward = (int) req.reward;
    ROS_INFO_STREAM("Received reward " << reward);
    q_learning->updateWeights(*game_state, reward);

    return true;
}

int main(int argc, char **argv)
{
    // start ros
    ros::init(argc, argv, "q_learning");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    BayesianGameState *game_state = new BayesianGameState();
    BehaviorAgent pacman;
    QLearning *q_learning = new QLearning;

    ros::Publisher chatter_pub = n.advertise<pacman_msgs::PacmanAction>("/pacman/pacman_action", 1000);
    ros::ServiceServer get_action_service = n.advertiseService<pacman_msgs::PacmanGetAction::Request, pacman_msgs::PacmanGetAction::Response>
                                ("/pacman/get_action", boost::bind(getAction, _1, _2, &game_state, pacman, q_learning));

    ros::ServiceServer receive_reward_service = n.advertiseService<pacman_msgs::RewardService::Request, pacman_msgs::RewardService::Response>
                                ("/pacman/reward", boost::bind(receiveReward, _1, _2, &game_state, q_learning));

    // client to start game service and server for end game service
    ros::ServiceClient start_game_client = n.serviceClient<pacman_msgs::StartGame>("/pacman/start_game");
    ros::ServiceServer end_game_service = n.advertiseService<pacman_msgs::EndGame::Request, pacman_msgs::EndGame::Response>
                                ("/pacman/end_game", boost::bind(endGame, _1, _2, &start_game_client, &game_state));
    ros::service::waitForService("/pacman/start_game", -1);

    // start first game
    pacman_msgs::StartGame start_game;
    if (NUMBER_OF_TRAININGS)
        start_game.request.show_gui = false;
    else
        start_game.request.show_gui = true;
    if (start_game_client.call(start_game))
    {
        if(start_game.response.started)
        {
            ROS_INFO("Game started");
        }
        else
        {
            ROS_ERROR("Failed to start game (check if game already started)");
        }
    }
    else // if problem print error
    {
        ROS_ERROR("Failed to call service StartGame");
    }

    // spin to answer services
    ros::spin();

    // shutdown ros node
    ros::shutdown();
}

// TODO: adicionar dois passos ou 0 para fantasmas
// TODO: adicionar dois passos e aumentar 0 para pacman
// TODO: update food probabilty
