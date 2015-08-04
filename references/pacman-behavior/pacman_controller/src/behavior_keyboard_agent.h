#include "ros/ros.h"
#include "pacman_interface/PacmanAction.h"
#include "std_msgs/String.h"

#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <string.h>

#include "pacman_agent.h"
#include "pacman_controller/game_info.h"
#include "util_functions.h"

using namespace std;

class BehaviorKeyboardAgent : public PacmanAgent
{
  private:
    typedef enum {STOP, EAT, EAT_BIG_FOOD, RUN, HUNT} Behaviors;

    std::string keyPressed;
    std::vector<string> validKeys;
    std::map<string, int> keyToBehavior;
    ros::Subscriber keypressSubscriber;
    
    void keypressCallback(const std_msgs::String::ConstPtr& msg);

    pacman_interface::PacmanAction getHuntAction(GameInfo game_info);
    pacman_interface::PacmanAction getRunAction(GameInfo game_info);
    pacman_interface::PacmanAction getEatBigFoodAction(GameInfo game_info);
    pacman_interface::PacmanAction getEatAction(GameInfo game_info);
    pacman_interface::PacmanAction getStopAction();

  public:
    BehaviorKeyboardAgent();
    void sendAction(GameInfo game_info);
    string getAgentName();
};