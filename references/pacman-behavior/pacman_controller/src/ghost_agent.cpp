#include "ghost_agent.h"

using namespace std;

int GhostAgent::number_of_ghost_agents_ = 0;

GhostAgent::GhostAgent()
{
    ghost_agent_number_ = number_of_ghost_agents_;
    number_of_ghost_agents_++;

    //throw std::logic_error("Agent class constructor wasn't meant to be used.");
    ROS_DEBUG("GhostAgent %d initialized", ghost_agent_number_);
}

int GhostAgent::getGhostNumber()
{
    return ghost_agent_number_;
}

string GhostAgent::getAgentName()
{
    return "GhostAgent";
}