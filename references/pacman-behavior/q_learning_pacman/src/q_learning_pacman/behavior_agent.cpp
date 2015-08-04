#include "q_learning_pacman/behavior_agent.h"

BehaviorAgent::BehaviorAgent()
{
    ROS_DEBUG("Behavior Agent initialized");
}

pacman_msgs::PacmanAction BehaviorAgent::getAction(BayesianGameState *game_state, int behavior)
{
    pacman_msgs::PacmanAction action;

    switch (behavior)
    {
        case STOP:
            action = getStopAction();
            ROS_INFO_STREAM("Stop behavior");
            break;
        case EAT:
            action = getEatAction(game_state);
            ROS_INFO_STREAM("Eat behavior");
            break;
        case EAT_BIG_FOOD:
            action = getEatBigFoodAction(game_state);
            ROS_INFO_STREAM("Eat big food behavior");
            break;
        case RUN:
            action = getRunAction(game_state);
            ROS_INFO_STREAM("Run behavior");
            break;
        case HUNT:
            action = getHuntAction(game_state);
            ROS_INFO_STREAM("Hunt behavior");
            break;
        default:
            action.action = action.STOP;
            ROS_ERROR_STREAM("Unknown default behavior");
            break;
    }

    return action;
}

std::string BehaviorAgent::getAgentName()
{
    return "BehaviorAgent";
}

pacman_msgs::PacmanAction BehaviorAgent::getHuntAction(BayesianGameState *game_state) {
    pacman_msgs::PacmanAction action;
    action.action = action.STOP;

    return action;
}

pacman_msgs::PacmanAction BehaviorAgent::getRunAction(BayesianGameState *game_state) {
    pacman_msgs::PacmanAction action;
    action.action = action.STOP;
    return action;
}

pacman_msgs::PacmanAction BehaviorAgent::getEatBigFoodAction(BayesianGameState *game_state) {
    pacman_msgs::PacmanAction action;
    action.action = action.STOP;
    return action;
}

pacman_msgs::PacmanAction BehaviorAgent::getEatAction(BayesianGameState *game_state) {
    pacman_msgs::PacmanAction action;
    action.action = action.STOP;
    return action;
}

pacman_msgs::PacmanAction BehaviorAgent::getStopAction() {
    pacman_msgs::PacmanAction action;
    action.action = action.STOP;
    return action;
}