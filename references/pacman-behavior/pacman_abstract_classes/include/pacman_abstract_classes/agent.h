#ifndef AGENT_H
#define AGENT_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

/**
 * Abstract class that implements an agent for the pacman game.
 * 
 * @author Tiago Pimentel Martins da Silva
 */
class Agent
{
  protected:
    ros::NodeHandle n_;
    geometry_msgs::Pose pose_;

    static geometry_msgs::Pose north_movement_;
    static geometry_msgs::Pose south_movement_;
    static geometry_msgs::Pose west_movement_;
    static geometry_msgs::Pose east_movement_;
    static geometry_msgs::Pose stop_movement_;
    static std::map<int, geometry_msgs::Pose> action_to_movement_;

  public:
    Agent();
    virtual void updatePosition();
    virtual std::string getAgentName();
};

#endif // AGENT_H