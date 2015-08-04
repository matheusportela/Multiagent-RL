#include "visible_ghost_agent.h"

using namespace std;

VisibleGhostAgent::VisibleGhostAgent()
{
    movement_subscriber_ = n_.subscribe<pacman_interface::AgentAction>("/pacman_interface/ghost_action", 1000, boost::bind(&VisibleGhostAgent::movementCallback, this, _1));

    pose_.position.x = 0;
    pose_.position.y = 0;
}

void VisibleGhostAgent::movementCallback(const pacman_interface::AgentAction::ConstPtr& action)
{
    if(action->agent == getGhostNumber())
    {
        geometry_msgs::Pose movement = action_to_movement_[action->action];

        pose_.position.x += movement.position.x;
        pose_.position.y += movement.position.y;
    }
}

string VisibleGhostAgent::getAgentName()
{
    return "VisibleGhostAgent";
}