#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "ros/ros.h"
#include <vector>

#include "particle_filter_pacman/game_particle.h"
#include "pacman_interface/PacmanAction.h"
#include "pacman_interface/AgentPose.h"
#include "geometry_msgs/Pose.h"

/**
 * Class that implements a particle filter on the pacman game.
 * 
 * @author Tiago Pimentel Martins da Silva
 */
class ParticleFilter
{
  public:
    ParticleFilter();

    void estimateMovement(pacman_interface::PacmanAction action);

    void printPacmanParticles();
    void printGhostParticles(int ghost_index);
    void printMostProbableMap();

    void estimateMap();

    bool hasNewObservation();
    void resetNewObservation();

    int getMapWidth();
    int getMapHeight();
    std::vector< std::vector<GameParticle::MapElements> > getEstimatedMap();
    geometry_msgs::Pose getEstimatedPacmanPose();
    std::vector< geometry_msgs::Pose > getEstimatedGhostsPoses();
    double getEstimatedScore();
    double getEstimatedReward();
    std::map< std::pair<int, int>, int > getDistances(int x, int y);
    
    std::vector< pacman_interface::PacmanAction > getLegalActions(int x, int y);
    std::vector< std::pair<int, int> > getLegalNextPositions(int x, int y);

  protected:
    ros::NodeHandle n_;
    ros::Subscriber ghost_distance_subscriber_;
    ros::Subscriber pacman_pose_subscriber_;
    std::vector< GameParticle > game_particles_;
    bool is_observed_;

    int map_height_;
    int map_width_;
    int num_ghosts_;
    std::vector< std::vector<bool> > walls_;
    double score_;
    double last_reward_;

    std::vector< std::vector<GameParticle::MapElements> > estimated_map_;
    geometry_msgs::Pose estimated_pacman_pose_;
    std::vector< geometry_msgs::Pose > estimated_ghosts_poses_;

    void sampleParticles(std::map< double, GameParticle > particles_map, double sum_prob_all_particles);
    void observePacman(const geometry_msgs::Pose::ConstPtr& msg);
    void observeGhost(const pacman_interface::AgentPose::ConstPtr& msg);

    void printPacmanOrGhostParticles(bool is_pacman, int ghost_index);

    std::map< std::pair<int, int>, std::map< std::pair<int, int>, int > > precalculated_distances_;
    std::map< std::pair<int, int>, int > calculateDistances(int x, int y);
    void precalculateAllDistances();
};

#endif // PARTICLE_FILTER_H