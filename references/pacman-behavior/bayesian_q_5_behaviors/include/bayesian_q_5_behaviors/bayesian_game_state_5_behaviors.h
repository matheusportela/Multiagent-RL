#ifndef BAYESIAN_GAME_STATE_H
#define BAYESIAN_GAME_STATE_H

#include "q_learning_pacman/game_state.h"

#include "geometry_msgs/Pose.h"
#include "pacman_msgs/AgentPoseService.h"

/**
 * Abstract class that implements a pacman agent for the pacman game.
 * 
 * @author Tiago Pimentel Martins da Silva
 */
class BayesianGameState : public GameState
{
  protected:
    void observeGhost(double measurement_x_dist, double measurement_y_dist, int ghost_index);
    void observePacman(double measurement_x, double measurement_y);
    bool observeAgent(pacman_msgs::AgentPoseService::Request &req, pacman_msgs::AgentPoseService::Response &res);
    bool is_finished_;

    ros::ServiceServer pacman_observer_service_;
    ros::ServiceServer ghost_distance_observer_service_;

    // precalculate all real distances in map
    std::map< std::pair<int, int>, std::map< std::pair<int, int>, int > > precalculated_distances_;
    std::map< std::pair<int, int>, int > calculateDistances(int x, int y);
    void precalculateAllDistances();

  public:
    BayesianGameState();
    ~BayesianGameState();
    
    void predictPacmanMove(pacman_msgs::PacmanAction action);
    void predictGhostMove(int ghost_index);
    void predictGhostsMoves();
    void predictAgentsMoves(pacman_msgs::PacmanAction action);

    // helper functions
    bool isFinished();
    float getClosestFoodDistance();
    float getClosestBigFoodDistance();
    bool eatsFood(pacman_msgs::PacmanAction action);
    int getClosestGhostDistance();
    int getNumberOfGhostsOneStepAway(pacman_msgs::PacmanAction action);
    int getNumberOfGhostsNStepsAway(int n);
    bool hasGhostNStepsAway(int n);
    std::pair< double, double > getProbabilityOfAGhosWhiteOrNotNStepsAway(int n);
    double getProbabilityOfAGhostNStepsAway(int n);
    double getProbabilityOfAWhiteGhostNStepsAway(int n);
    bool dies(pacman_msgs::PacmanAction action);
    double getProbOfWhiteGhosts();
    double getProbOfBigFood();

    float getMaxFoodProbability();
    float getMaxBigFoodProbability();
    geometry_msgs::Pose getMostProbablePacmanPose();
    geometry_msgs::Pose getMostProbableGhostPose(int ghost_index);
    std::vector< geometry_msgs::Pose > getMostProbableGhostsPoses();

    // usefull functions
    std::map< std::pair<int, int>, int > getDistances(int x, int y);
};

#endif // BAYESIAN_GAME_STATE_H