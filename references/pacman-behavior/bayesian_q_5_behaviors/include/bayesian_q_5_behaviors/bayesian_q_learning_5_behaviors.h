#include "ros/ros.h"
#include "pacman_interface/PacmanAction.h"

#include "bayesian_q_5_behaviors/bayesian_game_state_5_behaviors.h"

class BayesianQLearning
{
  protected:
    static int NUM_BEHAVIORS;
    static int NUM_FEATURES;
    static double learning_rate_;
    static double discount_factor_;
    static double exploration_rate_;
    static int num_training_; // number of training episodes, i.e. no learning after these many episodes
    static int no_exploration_training_matches_; // number of training episodes with no exploration

    std::clock_t begin_time_;

    std::vector<double> weights_;
    std::vector< std::vector<double> > behavioral_weights_;
    std::vector<double> features_;
    std::vector<double> temp_features_;
    std::vector<double> old_features_;

    std::vector<int> temp_per_match_chosen_behaviors_;
    std::vector< std::vector<int> > saved_per_match_chosen_behaviors_;
    std::vector<int> saved_chosen_behaviors_;
    std::vector<double> saved_match_scores_;
    std::vector<double> saved_match_time_diffs_;
    std::vector<double> saved_time_diffs_;
    std::vector< std::vector< std::vector<double> > > saved_match_behavioral_weights_;
    std::vector< std::vector< std::vector<double> > > saved_behavioral_weights_;
    void logScores(time_t time_now);
    void logWeights(std::vector< std::vector< std::vector<double> > > logged_weights, time_t time_now, bool log_per_match);
    void logChosenBehaviors(time_t time_now);
    void logEndOfMatchBehaviors(time_t time_now);
    void logTimes(time_t time_now);
    void saveWeights();

    double old_q_value_;
    double new_q_value_;
    double old_behavior_;
    double behavior_;
    std::vector<double> q_values_;

    void saveTempFeatures(int behavior);

    std::vector<double> getFeatures(BayesianGameState *game_state, int behavior);
    double getQValue(BayesianGameState *game_state, int behavior);

  public:
    BayesianQLearning();

    std::pair<int, double> getMaxQValue(BayesianGameState *game_state);
    void updateWeights(BayesianGameState *new_game_state, int reward);
    int getTrainingBehavior(BayesianGameState *game_state);
    int getBehavior(BayesianGameState *game_state);

    void saveWeightsToBeLogged();
    void saveMatchScore(int score);
    void saveEndOfMatchWeights();
    void logWeights();
};