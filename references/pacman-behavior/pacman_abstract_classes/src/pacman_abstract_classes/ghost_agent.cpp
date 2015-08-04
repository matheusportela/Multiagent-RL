#include "ghost_agent.h"

int GhostAgent::number_of_ghost_agents_ = 0;

GhostAgent::GhostAgent()
{
    ghost_agent_number_ = number_of_ghost_agents_;
    number_of_ghost_agents_++;

    ROS_DEBUG("GhostAgent %d initialized", ghost_agent_number_);
}

int GhostAgent::getGhostNumber()
{
    return ghost_agent_number_;
}

std::string GhostAgent::getAgentName()
{
    return "GhostAgent";
}