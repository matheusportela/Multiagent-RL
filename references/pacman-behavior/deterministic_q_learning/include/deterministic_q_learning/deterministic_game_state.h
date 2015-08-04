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
class DeterministicGameState : public GameState
{
  protected:
    void observeGhost(int measurement_x_dist, int measurement_y_dist, int ghost_index);
    void observePacman(int measurement_x, int measurement_y);
    bool observeAgent(pacman_msgs::AgentPoseService::Request &req, pacman_msgs::AgentPoseService::Response &res);
    bool is_finished_;

    ros::ServiceServer pacman_observer_service_;
    ros::ServiceServer ghost_distance_observer_service_;

    // precalculate all real distances in map
    std::map< std::pair<int, int>, std::map< std::pair<int, int>, int > > precalculated_distances_;
    std::map< std::pair<int, int>, int > calculateDistances(int x, int y);
    void precalculateAllDistances();

  public:
    DeterministicGameState();
    ~DeterministicGameState();
    
    void predictPacmanMove(pacman_msgs::PacmanAction action);
    void predictGhostMove(int ghost_index);
    void predictGhostsMoves();
    void predictAgentsMoves(pacman_msgs::PacmanAction action);

    // helper functions
    bool isActionLegal(int action);
    bool isActionLegal(pacman_msgs::PacmanAction action);
    geometry_msgs::Pose getNextPacmanPose(pacman_msgs::PacmanAction action);
    bool isFinished();
    float getClosestFoodDistance(pacman_msgs::PacmanAction action);
    bool eatsFood(pacman_msgs::PacmanAction action);
    int getClosestGhostDistance(pacman_msgs::PacmanAction action);
    int getNumberOfGhostsOneStepAway(pacman_msgs::PacmanAction action);
    int getNumberOfGhostsNStepsAway(int n);
    bool hasGhostNStepsAway(int n);
    bool dies(pacman_msgs::PacmanAction action);

    // usefull functions
    std::map< std::pair<int, int>, int > getDistances(int x, int y);
};

#endif // BAYESIAN_GAME_STATE_H