#ifndef GAME_PARTICLE_H
#define GAME_PARTICLE_H

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
class GameParticle
{
  public:
    GameParticle();
    typedef enum {EMPTY, FOOD, BIG_FOOD, WALL, ERROR} MapElements;
    pacman_interface::PacmanAction actions_;
    
    void printMap();

    int getHeight();
    int getWidth();
    MapElements getMapElement(int x, int y);
    int getScore();

	std::vector< pacman_interface::PacmanAction > getLegalActions(int x, int y);
	std::vector< std::pair<int, int> > getLegalNextPositions(int x, int y);
    void move(pacman_interface::PacmanAction action);

    std::vector< std::vector<MapElements> > getMap();

    geometry_msgs::Pose getPacmanPose();
    geometry_msgs::Pose getGhostPose(int ghost_index);
    std::vector< geometry_msgs::Pose > getGhostsPoses();
    int getNumberOfGhosts();

    std::vector<int> getWhiteGhostsTime();
    void checkIfDeadGhosts();

  protected:
    ros::NodeHandle n_;

    int height_;
    int width_;
    std::vector< std::vector<MapElements> > map_;
    int score_;

    std::vector< int > white_ghosts_time_;

    int num_ghosts_;
    geometry_msgs::Pose pacman_pose_;
    std::vector< geometry_msgs::Pose > ghosts_poses_;
    std::vector< geometry_msgs::Pose > spawn_ghosts_poses_;

    std::vector< std::pair< float, std::pair<int, int> > > getNextPositionsWithProbabilities(int x, int y, pacman_interface::PacmanAction action);

    void movePacman(pacman_interface::PacmanAction action);
    void moveGhost(std::vector< geometry_msgs::Pose >::reverse_iterator it);
    void moveGhosts();

    static bool particle_initialized;
};

#endif // GAME_PARTICLE_H