#ifndef NEW_GAME_INFO_H
#define NEW_GAME_INFO_H

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
class NewGameInfo
{
  public:
    NewGameInfo();
    typedef enum {EMPTY, FOOD, BIG_FOOD, WALL, ERROR} MapElements;
    pacman_interface::PacmanAction actions_;
    
    void printMap();
    void printMap2();

    int getHeight();
    int getWidth();
    MapElements getMapElement(int x, int y);

    std::vector< pacman_interface::PacmanAction > getLegalActions(int x, int y);
    std::vector< std::pair<int, int> > getLegalNextPositions(int x, int y);
    std::vector< std::pair< float, std::pair<int, int> > > getNextPositionsForActionWithProbabilities(int x, int y, pacman_interface::PacmanAction action);

    geometry_msgs::Pose getPacmanPose();
    std::vector< geometry_msgs::Pose > getGhostsPoses();
    int getNumberOfGhosts();

    std::vector< std::vector<float> > getPacmanPoseMap();
    std::vector< std::vector<float> > getGhostPoseMap(int ghost_index);
    void setPacmanPoseMap(std::vector< std::vector<float> > pacman_pose_map);
    void setGhostPoseMap(std::vector< std::vector<float> > ghost_pose_map, int ghost_index);

    static int MAX_DISTANCE;

    void printPacmanOrGhostPose( bool is_pacman, int ghost_index);

  protected:
    ros::NodeHandle n_;
    ros::Subscriber update_world_subscriber_;
    geometry_msgs::Pose pose_;

    int height_;
    int width_;
    std::vector< std::vector<MapElements> > map_;

    int num_ghosts_;
    geometry_msgs::Pose pacman_pose_;
    geometry_msgs::Pose pacman_pose_sd_;
    std::vector< geometry_msgs::Pose > ghosts_poses_;
    std::vector< geometry_msgs::Pose > ghosts_poses_sd_;

    std::vector< std::vector<float> > pacman_pose_map_;
    std::vector< std::vector< std::vector<float> > > ghosts_poses_map_;
    std::vector< std::vector<float> > foods_map_;
};

#endif // NEW_GAME_INFO_H