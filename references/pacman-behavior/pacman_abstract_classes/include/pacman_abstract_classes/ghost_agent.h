#ifndef GHOST_AGENT_H
#define GHOST_AGENT_H

#include "agent.h"

/**
 * Abstract class that implements a ghost agent for the pacman game.
 * 
 * @author Tiago Pimentel Martins da Silva
 * @extends Agent
 */
class GhostAgent : public Agent
{
  protected:
    static int number_of_ghost_agents_;
    int ghost_agent_number_;

  public:
    GhostAgent();
    int getGhostNumber();
    std::string getAgentName();
};



#endif // GHOST_AGENT_H