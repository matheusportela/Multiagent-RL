#include "particle_filter_pacman/q_learning.h"

int QLearning::NUM_BEHAVIORS = 5;
int QLearning::NUM_FEATURES = 7;
double QLearning::learning_rate_ = 0.5;
double QLearning::discount_factor_ = 0.95;
double QLearning::exploration_rate_ = 0.5;
int QLearning::num_training_ = 10;

QLearning::QLearning()
{

}

std::vector<double> QLearning::getFeatures(ParticleFilter *particle_filter)
{
    throw std::logic_error("The method getFeatures() is not implemented for base class QLearning.");
}

void QLearning::updateFeatures(ParticleFilter *particle_filter)
{
    features_ = getFeatures(particle_filter);
}

double QLearning::getQValue(int behavior)
{
    double q_value = 0;

    std::vector<double>::iterator features_it = features_.begin();
    std::vector<double>::iterator weights_it = weights_.begin();
    for(; features_it != features_.end() ; ++features_it, ++weights_it)
    {
        q_value += *features_it * *weights_it;
    }

    return q_value;
}

int QLearning::getMaxQValue()
{
    int behavior = -1;
    double q_value = 0;
    double max_q_value = 0;

    for(int i = 0; i < NUM_BEHAVIORS ; ++i)
    {
        q_value = getQValue(i);
        if(q_value > max_q_value)
        {
            max_q_value = q_value;
            behavior = i;
        }
    }
    
    old_q_value_ = max_q_value;

    return behavior;
}

int QLearning::getBehavior()
{
    int behavior = -1;
    double q_value = 0;
    double max_q_value = 0;

    for(int i = 0; i < NUM_BEHAVIORS ; ++i)
    {
        q_value = getQValue(i);
        if(q_value > max_q_value)
        {
            max_q_value = q_value;
            behavior = i;
        }
    }

    old_q_value_ = max_q_value;

    return behavior;
}

void QLearning::updateWeights(int reward)
{
    std::vector<double>::iterator features_it = features_.begin();
    std::vector<double>::iterator weights_it = weights_.begin();

    double error = 1; // reward + discount_factor * q_value(next_state) - q_value(this_state) // TODO: add this

    for(; features_it != features_.end() ; ++features_it, ++weights_it)
    {
        *weights_it = *weights_it + learning_rate_ * /*TODO: error */ *features_it;
    }
}