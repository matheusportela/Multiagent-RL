#include "ros/ros.h"
#include "pacman_msgs/PacmanAction.h"

#include "pacman_abstract_classes/pacman_agent.h"
#include "pacman_abstract_classes/util_functions.h"
#include "q_learning_pacman/bayesian_game_state.h"

class BehaviorAgent : public PacmanAgent
{
  private:
    typedef enum {STOP, EAT, EAT_BIG_FOOD, RUN, HUNT} Behaviors;

    pacman_msgs::PacmanAction getHuntAction(BayesianGameState *game_state);
    pacman_msgs::PacmanAction getRunAction(BayesianGameState *game_state);
    pacman_msgs::PacmanAction getEatBigFoodAction(BayesianGameState *game_state);
    pacman_msgs::PacmanAction getEatAction(BayesianGameState *game_state);
    pacman_msgs::PacmanAction getStopAction();

  public:
    BehaviorAgent();
    pacman_msgs::PacmanAction getAction(BayesianGameState *game_state, int behavior);
    std::string getAgentName();
};