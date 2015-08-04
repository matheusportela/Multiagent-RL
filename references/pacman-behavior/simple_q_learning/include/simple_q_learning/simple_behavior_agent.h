#include "ros/ros.h"
#include "pacman_msgs/PacmanAction.h"

#include "pacman_abstract_classes/pacman_agent.h"
#include "pacman_abstract_classes/util_functions.h"
#include "simple_q_learning/simple_game_state.h"

class SimplePacmanAgent : public PacmanAgent
{
  private:
    typedef enum {STOP, EAT, EAT_BIG_FOOD, RUN, HUNT} Behaviors;
    static int NUMBER_OF_BEHAVIORS_;

    pacman_msgs::PacmanAction getWestAction(DeterministicGameState *game_state);
    pacman_msgs::PacmanAction getEastAction(DeterministicGameState *game_state);
    pacman_msgs::PacmanAction getNorthAction(DeterministicGameState *game_state);
    pacman_msgs::PacmanAction getSouthAction(DeterministicGameState *game_state);
    pacman_msgs::PacmanAction getStopAction();

  public:
    SimplePacmanAgent();
    pacman_msgs::PacmanAction getAction(DeterministicGameState *game_state, int behavior);
    std::string getAgentName();
};