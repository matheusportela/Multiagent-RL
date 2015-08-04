#include "particle_filter_pacman/q_learning.h"

class QLearningSimple : public QLearning
{
  private:

    std::vector<double> getFeatures(ParticleFilter *particle_filter);
    double getQValue(int behavior);
    
  public:
    QLearningSimple();

    void updateFeatures(ParticleFilter *particle_filter);
    int getMaxQValue();
    void updateWeights(int reward);
    int getBehavior();
    
};