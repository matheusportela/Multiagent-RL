#include "pacman_abstract_classes/agent.h"

#include "pacman_abstract_classes/util_functions.h"
#include "pacman_interface/PacmanAction.h"

geometry_msgs::Pose Agent::north_movement_;
geometry_msgs::Pose Agent::south_movement_;
geometry_msgs::Pose Agent::west_movement_;
geometry_msgs::Pose Agent::east_movement_;
geometry_msgs::Pose Agent::stop_movement_;
std::map<int, geometry_msgs::Pose> Agent::action_to_movement_;

Agent::Agent()
{
    north_movement_.position = util::createPoint(0, 1, 0);
    south_movement_.position = util::createPoint(0, -1, 0);
    west_movement_.position = util::createPoint(1, 0, 0);
    east_movement_.position = util::createPoint(-1, 0, 0);
    stop_movement_.position = util::createPoint(0, 0, 0);
    action_to_movement_[pacman_interface::PacmanAction::NORTH] = north_movement_;
    action_to_movement_[pacman_interface::PacmanAction::SOUTH] = south_movement_;
    action_to_movement_[pacman_interface::PacmanAction::WEST] = west_movement_;
    action_to_movement_[pacman_interface::PacmanAction::EAST] = east_movement_;
    action_to_movement_[pacman_interface::PacmanAction::STOP] = east_movement_;
}

void Agent::updatePosition()
{
    throw std::logic_error("The method or operation is not implemented.");
}

std::string Agent::getAgentName()
{
    return "Agent";
}