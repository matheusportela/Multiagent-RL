#include "ros/ros.h"

#include "pacman_msgs/PacmanAction.h"
#include "pacman_msgs/StartGame.h"
#include "pacman_msgs/EndGame.h"
#include "pacman_msgs/PacmanGetAction.h"
#include "pacman_msgs/RewardService.h"

#include "bayesian_q_5_behaviors/bayesian_game_state_5_behaviors.h"
#include "bayesian_q_5_behaviors/bayesian_5_behaviors_agent.h"
#include "bayesian_q_5_behaviors/bayesian_q_learning_5_behaviors.h"

#include <mcheck.h>

int NUMBER_OF_GAMES_WITH_NO_GUI = 0;
int NUMBER_OF_GAMES = 2000;
int NUMBER_OF_TRAININGS = 0;
bool is_training = true;

bool endGame(pacman_msgs::EndGame::Request &req, pacman_msgs::EndGame::Response &res, 
        ros::ServiceClient *start_game_client, BayesianGameState **game_state, BayesianQLearning *q_learning)
{
    // count number of games
    static int game_count = 0;
    game_count++;

    // save scores and learning weights in end of match
    int match_score = (int) req.score;
    q_learning->saveMatchScore(match_score);
    q_learning->saveEndOfMatchWeights();

    if (req.win)
        ROS_INFO_STREAM("Won game " << game_count);
    else
        ROS_WARN_STREAM("Lost game " << game_count);

    if (game_count < NUMBER_OF_GAMES)
    {
        pacman_msgs::StartGame start_game;

        if (game_count >= NUMBER_OF_TRAININGS)
            is_training = false;

        if (game_count < NUMBER_OF_GAMES_WITH_NO_GUI)
            start_game.request.show_gui = false;
        else
            start_game.request.show_gui = true;

        if (start_game_client->call(start_game))
            if(start_game.response.started)
            {
                // new game started
                delete *game_state;
                *game_state = new BayesianGameState;
                res.game_restarted = true;

                //ROS_INFO("New game started");
                return true;
            }
            else
                ROS_ERROR("Failed to start game (check if game already started)");
        else // if problem => print error
            ROS_ERROR("Failed to call service StartGame");
    }
    else
        ROS_INFO_STREAM("Game finished, type ctrl+c to exit");

    // game not restarted
    res.game_restarted = false;

    return true;
}

bool getAction(pacman_msgs::PacmanGetAction::Request &req, pacman_msgs::PacmanGetAction::Response &res, 
                    BayesianGameState **game_state, BayesianBehaviorAgent pacman, BayesianQLearning *q_learning)
{
    int behavior;

    //ROS_INFO_STREAM("Sending action");

    sleep(10);

    // predict next game state
    if (is_training) {
        behavior = q_learning->getTrainingBehavior(*game_state);
    } else {
        behavior = q_learning->getBehavior(*game_state);
    }
    //ROS_INFO_STREAM("Getting action");


    pacman_msgs::PacmanAction action = pacman.getAction(*game_state, behavior);
    //ROS_INFO_STREAM("Predicting movement");
    (*game_state)->predictAgentsMoves(action);

    res.action = action.action;

    //ROS_INFO_STREAM("Done sending agent");

    return true;
}

bool receiveReward(pacman_msgs::RewardService::Request &req, pacman_msgs::RewardService::Response &res, 
                    BayesianGameState **game_state, BayesianQLearning *q_learning)
{
    int reward = (int) req.reward;
    //ROS_INFO_STREAM("Received reward " << reward);

    if (is_training) {
        q_learning->updateWeights(*game_state, reward);
    } else {
        q_learning->saveWeightsToBeLogged();
    }

    return true;
}

int main(int argc, char **argv)
{
    // start ros
    ros::init(argc, argv, "q_learning");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    srand (time(NULL)); // start random fucntions
    BayesianGameState *game_state = new BayesianGameState;
    BayesianBehaviorAgent pacman;
    BayesianQLearning *q_learning = new BayesianQLearning;

    ros::Publisher chatter_pub = n.advertise<pacman_msgs::PacmanAction>("/pacman/pacman_action", 1000);
    ros::ServiceServer get_action_service = n.advertiseService<pacman_msgs::PacmanGetAction::Request, pacman_msgs::PacmanGetAction::Response>
                                ("/pacman/get_action", boost::bind(getAction, _1, _2, &game_state, pacman, q_learning));

    ros::ServiceServer receive_reward_service = n.advertiseService<pacman_msgs::RewardService::Request, pacman_msgs::RewardService::Response>
                                ("/pacman/reward", boost::bind(receiveReward, _1, _2, &game_state, q_learning));

    // client to start game service and server for end game service
    ros::ServiceClient start_game_client = n.serviceClient<pacman_msgs::StartGame>("/pacman/start_game");
    ros::ServiceServer end_game_service = n.advertiseService<pacman_msgs::EndGame::Request, pacman_msgs::EndGame::Response>
                                ("/pacman/end_game", boost::bind(endGame, _1, _2, &start_game_client, &game_state, q_learning));
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
            //ROS_INFO("Game started");
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

    //q_learning->logWeights();

    // shutdown ros node
    ros::shutdown();
}

// Working for deterministic behavioral games

// TODO: uncomment q_learnings