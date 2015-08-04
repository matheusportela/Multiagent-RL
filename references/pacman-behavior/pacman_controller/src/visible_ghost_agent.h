#ifndef VISIBLE_GHOST_AGENT_H
#define VISIBLE_GHOST_AGENT_H

#include "ghost_agent.h"

#include "pacman_interface/AgentAction.h"

/**
 * Class that implements a visible ghost agent for the pacman game.
 * This ghost agent has no error in its position.
 * 
 * @author Tiago Pimentel Martins da Silva
 */
class VisibleGhostAgent : public GhostAgent
{
  protected:
    ros::Subscriber movement_subscriber_;

    void movementCallback(const pacman_interface::AgentAction::ConstPtr& action);

  public:
    VisibleGhostAgent();
    std::string getAgentName();
};

#endif // VISIBLE_GHOST_AGENT_H