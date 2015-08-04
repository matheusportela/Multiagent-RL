#include "ros/ros.h"
#include "pacman_msgs/PacmanAction.h"

#include "pacman_abstract_classes/pacman_agent.h"
#include "pacman_abstract_classes/util_functions.h"
#include "bayesian_q_5_behaviors/bayesian_game_state_5_behaviors.h"

class BayesianBehaviorAgent : public PacmanAgent
{
  private:
    typedef enum {STOP, EAT, RUN, EAT_BIG_FOOD, HUNT} Behaviors;
    static int NUMBER_OF_BEHAVIORS_;

    pacman_msgs::PacmanAction getWestAction(BayesianGameState *game_state);
    pacman_msgs::PacmanAction getEastAction(BayesianGameState *game_state);
    pacman_msgs::PacmanAction getNorthAction(BayesianGameState *game_state);
    pacman_msgs::PacmanAction getSouthAction(BayesianGameState *game_state);
    pacman_msgs::PacmanAction getHuntAction(BayesianGameState *game_state);
    pacman_msgs::PacmanAction getRunAction(BayesianGameState *game_state);
    pacman_msgs::PacmanAction getEatBigFoodAction(BayesianGameState *game_state);
    pacman_msgs::PacmanAction getEatAction(BayesianGameState *game_state);
    pacman_msgs::PacmanAction getStopAction();

  public:
    BayesianBehaviorAgent();
    pacman_msgs::PacmanAction getAction(BayesianGameState *game_state, int behavior);
    std::string getAgentName();
};