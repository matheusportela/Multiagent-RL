#ifndef GAME_H
#define GAME_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "pacman_interface/PacmanAction.h"
#include "pacman_interface/AgentAction.h"
#include "std_msgs/String.h"

#include <vector>


/**
 * Class that holds information on the pacman game.
 * 
 * @author Tiago Pimentel Martins da Silva
 */
class GameInfo
{
  public:
    GameInfo();
    typedef enum {EMPTY, FOOD, BIG_FOOD, WALL, ERROR} MapElements;
    pacman_interface::PacmanAction actions_;
    
    void printMap();

    int getHeight();
    int getWidth();
    MapElements getMapElement(int x, int y);

	std::vector< pacman_interface::PacmanAction > getLegalActions(int x, int y);
	std::vector< std::pair<int, int> > getLegalNextPositions(int x, int y);

    void precalculateAllDistances();
    std::map< std::pair<int, int>, int > getDistances(int x, int y);

    geometry_msgs::Pose getPacmanPose();
    std::vector< geometry_msgs::Pose > getGhostsPoses();
    int getNumberOfGhosts();

    static int MAX_DISTANCE;

  protected:
    ros::NodeHandle n_;
    ros::Subscriber update_world_subscriber_;
    geometry_msgs::Pose pose_;

    int height_;
    int width_;
    std::vector< std::vector<MapElements> > map_;

    int num_ghosts_;
    geometry_msgs::Pose pacman_pose_;
    std::vector< geometry_msgs::Pose > ghosts_poses_;

    void updateAgents(const pacman_interface::AgentAction::ConstPtr& msg);

    std::map< std::pair<int, int>, int > calculateDistances(int x, int y);
    std::map< std::pair<int, int>, std::map< std::pair<int, int>, int > > precalculated_distances_;
};

#endif // GAME_H