#include "pacman_abstract_classes/pacman_agent.h"

#include "pacman_interface/PacmanAction.h"

PacmanAgent::PacmanAgent()
{
    action_publisher_ = n_.advertise<pacman_interface::PacmanAction>("/pacman_interface/pacman_action", 1000);
    ROS_DEBUG("PacmanAgent initialized");
}

void PacmanAgent::sendAction()
{
    throw std::logic_error("The method or operation is not implemented.");
}

std::string PacmanAgent::getAgentName()
{
    return "PacmanAgent";
}