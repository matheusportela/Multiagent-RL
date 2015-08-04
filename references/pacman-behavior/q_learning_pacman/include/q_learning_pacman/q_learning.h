#include "ros/ros.h"
#include "pacman_interface/PacmanAction.h"

#include "q_learning_pacman/bayesian_game_state.h"

class QLearning
{
  protected:
    static int NUM_BEHAVIORS;
    static int NUM_FEATURES;
    static double learning_rate_;
    static double discount_factor_;
    static double exploration_rate_;
    static int num_training_; // number of training episodes, i.e. no learning after these many episodes

    std::vector<double> weights_;
    std::vector<double> features_;
    std::vector<double> old_features_;

    double old_q_value_;
    double new_q_value_;
    double old_behavior_;
    double behavior_;
    std::vector<double> q_values_;

    std::vector<double> getFeatures(BayesianGameState *game_state);
    double getQValue(BayesianGameState *game_state, int behavior);

  public:
    QLearning();

    std::pair<int, double> getMaxQValue(BayesianGameState *game_state);
    void updateWeights(BayesianGameState *new_game_state, int reward);
    int getBehavior(BayesianGameState *game_state);
};