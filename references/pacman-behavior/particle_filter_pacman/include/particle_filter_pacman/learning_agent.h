#include "ros/ros.h"
#include "pacman_interface/PacmanAction.h"

#include "pacman_abstract_classes/pacman_agent.h"
#include "pacman_abstract_classes/util_functions.h"
#include "particle_filter_pacman/particle_filter.h"

class LearningAgent : public PacmanAgent
{
  private:
    typedef enum {STOP, EAT, EAT_BIG_FOOD, RUN, HUNT} Behaviors;

    pacman_interface::PacmanAction getHuntAction(ParticleFilter *particle_filter);
    pacman_interface::PacmanAction getRunAction(ParticleFilter *particle_filter);
    pacman_interface::PacmanAction getEatBigFoodAction(ParticleFilter *particle_filter);
    pacman_interface::PacmanAction getEatAction(ParticleFilter *particle_filter);
    pacman_interface::PacmanAction getStopAction();

  public:
    LearningAgent();
    pacman_interface::PacmanAction sendAction(ParticleFilter *particle_filter, int behavior);
    std::string getAgentName();
};