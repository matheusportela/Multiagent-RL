#include "ros/ros.h"
#include "pacman_interface/PacmanAction.h"
#include "std_msgs/String.h"

#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <string.h>

#include "pacman_agent.h"

using namespace std;

class KeyboardAgent : public PacmanAgent
{
  private:
    std::string keyPressed;
    std::vector<string> validKeys;
    std::map<string, int> keyToAction;
    ros::Subscriber keypressSubscriber;
    
    void keypressCallback(const std_msgs::String::ConstPtr& msg);

  public:
    KeyboardAgent();
    void sendAction();
    string getAgentName();
};