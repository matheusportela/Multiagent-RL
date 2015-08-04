#include "ros/ros.h"
#include "pacman_interface/PacmanAction.h"

#include "particle_filter_pacman/particle_filter.h"

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

    virtual std::vector<double> getFeatures(ParticleFilter *particle_filter);
    double getQValue(int behavior);
    int getMaxQValue();

  public:
    QLearning();

    void updateFeatures(ParticleFilter *particle_filter);
    void updateWeights(int reward);
    int getBehavior();
};