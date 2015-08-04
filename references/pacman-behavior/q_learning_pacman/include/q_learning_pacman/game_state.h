#ifndef GAME_STATE_H
#define GAME_STATE_H

#include "ros/ros.h"
#include <vector>

#include "geometry_msgs/Pose.h"
#include "pacman_msgs/PacmanAction.h"

/**
 * Class that holds information on the pacman game.
 * 
 * @author Tiago Pimentel Martins da Silva
 */
class GameState
{
  public:
    GameState();
    ~GameState();
    typedef enum {EMPTY, FOOD, BIG_FOOD, WALL, ERROR} MapElements;
    
    void printMap();
    void printDeterministicMap();

    int getHeight();
    int getWidth();
    MapElements getMapElement(int x, int y);

    std::vector< pacman_msgs::PacmanAction > getLegalActions();
    std::vector< pacman_msgs::PacmanAction > getLegalActions(int x, int y);
    std::vector< std::pair<int, int> > getLegalNextPositions(int x, int y);
    std::vector< std::pair< float, std::pair<int, int> > > getNextPositionsForActionWithProbabilities(int x, int y, pacman_msgs::PacmanAction action);

    int getNumberOfGhosts();

    geometry_msgs::Pose getPacmanPose();
    geometry_msgs::Pose getGhostPose(int ghost_index);
    std::vector< geometry_msgs::Pose > getGhostsPoses();
    std::vector< std::vector<float> > getPacmanPoseMap();
    std::vector< std::vector<float> > getGhostPoseMap(int ghost_index);
    std::vector< std::vector< std::vector<float> > > getGhostsPoseMaps();
    std::vector< std::vector<float> > getFoodMap();
    std::vector< std::vector<float> > getBigFoodMap();
    void setPacmanPoseMap(std::vector< std::vector<float> > pacman_pose_map);
    void setGhostPoseMap(std::vector< std::vector<float> > ghost_pose_map, int ghost_index);

    static int MAX_DISTANCE;

    void printPacmanOrGhostPose( bool is_pacman, int ghost_index);
    void printFoodsMap();
    void printBigFoodsMap();
    void printWhiteGhostsProbabilities();

  protected:
    ros::NodeHandle n_;

    int height_;
    int width_;
    std::vector< std::vector<MapElements> > map_;

    int num_ghosts_;
    // probabilistic variables
    std::vector< std::vector<float> > pacman_pose_map_;
    std::vector< std::vector< std::vector<float> > > ghosts_poses_map_;
    std::vector< std::vector<float> > foods_map_;
    std::vector< std::vector<float> > big_foods_map_;
    std::vector< std::vector<float> > probability_ghosts_white_;

    std::vector< geometry_msgs::Pose > ghosts_spawn_poses_;

    // deterministic variables
    geometry_msgs::Pose pacman_pose_;
    std::vector< geometry_msgs::Pose > ghosts_poses_;
};

#endif // GAME_STATE_H