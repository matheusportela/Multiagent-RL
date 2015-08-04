#include "simple_q_learning/simple_behavior_agent.h"

int SimplePacmanAgent::NUMBER_OF_BEHAVIORS_ = 5;

SimplePacmanAgent::SimplePacmanAgent()
{
    ROS_DEBUG("Behavior Agent initialized");
}

pacman_msgs::PacmanAction SimplePacmanAgent::getAction(DeterministicGameState *game_state, int behavior)
{
    pacman_msgs::PacmanAction action;

    switch (behavior)
    {
        case pacman_msgs::PacmanAction::STOP:
            action = getStopAction();
            ROS_DEBUG_STREAM("Stop behavior");
            break;
        case pacman_msgs::PacmanAction::WEST:
            action = getWestAction(game_state);
            ROS_DEBUG_STREAM("West behavior");
            break;
        case pacman_msgs::PacmanAction::EAST:
            action = getEastAction(game_state);
            ROS_DEBUG_STREAM("East behavior");
            break;
        case pacman_msgs::PacmanAction::NORTH:
            action = getNorthAction(game_state);
            ROS_DEBUG_STREAM("North behavior");
            break;
        case pacman_msgs::PacmanAction::SOUTH:
            action = getSouthAction(game_state);
            ROS_DEBUG_STREAM("South behavior");
            break;
        default:
            action.action = action.STOP;
            ROS_ERROR_STREAM("Unknown default behavior");
            break;
    }

    return action;
}

std::string SimplePacmanAgent::getAgentName()
{
    return "SimplePacmanAgent";
}

pacman_msgs::PacmanAction SimplePacmanAgent::getWestAction(DeterministicGameState *game_state) {
    pacman_msgs::PacmanAction action;
    action.action = action.WEST;

    return action;
}

pacman_msgs::PacmanAction SimplePacmanAgent::getEastAction(DeterministicGameState *game_state) {
    pacman_msgs::PacmanAction action;
    action.action = action.EAST;
    return action;
}

pacman_msgs::PacmanAction SimplePacmanAgent::getNorthAction(DeterministicGameState *game_state) {
    pacman_msgs::PacmanAction action;
    action.action = action.NORTH;
    return action;
}

pacman_msgs::PacmanAction SimplePacmanAgent::getSouthAction(DeterministicGameState *game_state) {
    pacman_msgs::PacmanAction action;
    action.action = action.SOUTH;
    return action;
}

pacman_msgs::PacmanAction SimplePacmanAgent::getStopAction() {
    pacman_msgs::PacmanAction action;
    action.action = action.STOP;
    return action;
}