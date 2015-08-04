#include "ros/ros.h"
#include "pacman_interface/PacmanAction.h"

#include "deterministic_q_learning/deterministic_game_state.h"

class DeterministicQLearning
{
  protected:
    static int NUM_BEHAVIORS;
    static int NUM_FEATURES;
    static double learning_rate_;
    static double discount_factor_;
    static double exploration_rate_;
    static int num_training_; // number of training episodes, i.e. no learning after these many episodes

    std::vector<double> weights_;
    std::vector< std::vector<double> > behavioral_weights_;
    std::vector<double> features_;
    std::vector<double> temp_features_;
    std::vector<double> old_features_;

    double old_q_value_;
    double new_q_value_;
    double old_behavior_;
    double behavior_;
    std::vector<double> q_values_;

    void saveTempFeatures(int behavior);

    std::vector<double> getFeatures(DeterministicGameState *game_state, int behavior);
    double getQValue(DeterministicGameState *game_state, int behavior);

  public:
    DeterministicQLearning();

    std::pair<int, double> getMaxQValue(DeterministicGameState *game_state);
    void updateWeights(DeterministicGameState *new_game_state, int reward);
    int getTrainingBehavior(DeterministicGameState *game_state);
    int getBehavior(DeterministicGameState *game_state);
};