#include "ros/ros.h"
#include "pacman_interface/PacmanAction.h"
#include "std_msgs/String.h"

#include "pacman_abstract_classes/pacman_agent.h"
#include "pacman_abstract_classes/util_functions.h"
#include "particle_filter_pacman/particle_filter.h"

class BehaviorKeyboardAgent : public PacmanAgent
{
  private:
    typedef enum {STOP, EAT, EAT_BIG_FOOD, RUN, HUNT} Behaviors;

    std::string keyPressed;
    std::vector < std::string > validKeys;
    std::map < std::string, int > keyToBehavior;
    ros::Subscriber keypressSubscriber;
    
    void keypressCallback(const std_msgs::String::ConstPtr& msg);

    pacman_interface::PacmanAction getHuntAction(ParticleFilter *particle_filter);
    pacman_interface::PacmanAction getRunAction(ParticleFilter *particle_filter);
    pacman_interface::PacmanAction getEatBigFoodAction(ParticleFilter *particle_filter);
    pacman_interface::PacmanAction getEatAction(ParticleFilter *particle_filter);
    pacman_interface::PacmanAction getStopAction();

  public:
    BehaviorKeyboardAgent();
    pacman_interface::PacmanAction sendAction(ParticleFilter *particle_filter);
    std::string getAgentName();
};