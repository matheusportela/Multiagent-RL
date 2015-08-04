#include "deterministic_q_learning/deterministic_q_learning.h"
#include "pacman_abstract_classes/util_functions.h"

#include "deterministic_q_learning/deterministic_behavior_agent.h"

int DeterministicQLearning::NUM_BEHAVIORS = 5;
int DeterministicQLearning::NUM_FEATURES = 7;
double DeterministicQLearning::learning_rate_ = 0.002;
double DeterministicQLearning::discount_factor_ = 0.99;
double DeterministicQLearning::exploration_rate_ = 0.8;
int DeterministicQLearning::num_training_ = 10;

DeterministicQLearning::DeterministicQLearning()
{
    //features_= std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    //old_features_= std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    //weights_ = std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    std::vector<double> temp_weights = std::vector<double> (NUM_FEATURES, 0);
    behavioral_weights_ = std::vector< std::vector<double> > (NUM_BEHAVIORS, temp_weights);
    old_q_value_ = 0;
    new_q_value_ = 0;
    old_behavior_ = 0;
    behavior_ = 0;
}

void DeterministicQLearning::saveTempFeatures(int behavior)
{
    features_ = temp_features_;
    behavior_ = behavior;
}

std::vector<double> DeterministicQLearning::getFeatures(DeterministicGameState *game_state, int behavior)
{
    std::vector<double> features;

    // TODO: get features
    features.push_back(1.0); // bias

    pacman_msgs::PacmanAction action;
    DeterministicBehaviorAgent pacman_agent;
    action = pacman_agent.getAction(game_state, behavior);

    // get distances can't be manhattan
    //features.push_back( game_state->eatsFood(action) / 10.0 );
    features.push_back( game_state->getClosestFoodDistance(action) / ( 1.0 * game_state->getHeight() * game_state->getWidth() ) );
    //features.push_back( game_state->getNumberOfGhostsOneStepAway(action) / 10.0 );
    //features.push_back( game_state->getNumberOfGhostsNStepsAway(2) );
    features.push_back( 1.0/game_state->getClosestGhostDistance(action) );
    //features.push_back( game_state->dies(action) );

    return features;
}

double DeterministicQLearning::getQValue(DeterministicGameState *game_state, int behavior)
{
    double q_value = 0;

    temp_features_ = getFeatures(game_state, behavior);
    std::vector<double>::iterator features_it = temp_features_.begin();
    //std::vector<double>::iterator weights_it = weights_.begin();
    std::vector<double> weights = behavioral_weights_[behavior];
    std::vector<double>::iterator weights_it = weights.begin();
    for(int i = 0; features_it != temp_features_.end() && weights_it != weights.end() ; ++i, ++features_it, ++weights_it)
    {
        q_value += *features_it * *weights_it;
        //ROS_INFO_STREAM(" - - - " << i << " q value " << q_value << " features_ " << *features_it << " weight " << *weights_it);
    }
    //ROS_INFO_STREAM(" - - q value " << q_value << " for behavior " << behavior);

    return q_value;
}

std::pair<int, double> DeterministicQLearning::getMaxQValue(DeterministicGameState *game_state)
{
    int behavior = -1;
    double q_value = 0;
    double max_q_value = - util::INFINITE;

    for(int i = 0; i < NUM_BEHAVIORS ; ++i)
    {
        q_value = getQValue(game_state, i);

        if(q_value > max_q_value)
        {
            max_q_value = q_value;
            behavior = i;
            saveTempFeatures(behavior);
        }
    }

    if (behavior == 3)
        ROS_ERROR_STREAM("max q behavior " << behavior << " with value " << max_q_value);
    else
        ROS_INFO_STREAM("max q behavior " << behavior << " with value " << max_q_value);

    return std::make_pair(behavior, max_q_value);
}

int DeterministicQLearning::getBehavior(DeterministicGameState *game_state)
{
    std::pair<int, double> behavior_q_value_pair = getMaxQValue(game_state);
    int behavior = behavior_q_value_pair.first;
    old_q_value_ = behavior_q_value_pair.second;
    return behavior;
}

int DeterministicQLearning::getTrainingBehavior(DeterministicGameState *game_state)
{
    double random = rand() / (float) RAND_MAX;
    if (random < exploration_rate_) {
        int behavior = rand() % NUM_BEHAVIORS;

        old_q_value_ = getQValue(game_state, behavior);
        saveTempFeatures(behavior);

        if (behavior == 3)
            ROS_ERROR_STREAM("random behavior " << behavior);
        else
            ROS_INFO_STREAM("random behavior " << behavior);

        return behavior;
    }
    else
        return getBehavior(game_state);
}

void DeterministicQLearning::updateWeights(DeterministicGameState *new_game_state, int reward)
{
    old_features_ = features_;
    old_behavior_ = behavior_;
    if (new_game_state->isFinished())
    {
        new_q_value_ = 0;
    }
    else
    {
        std::pair<int, double> behavior_q_value_pair = getMaxQValue(new_game_state);
        new_q_value_ = behavior_q_value_pair.second;
    }

    // error = reward + discount_factor * q_value(new_state) - q_value(old_state)
    double error = reward + discount_factor_ * new_q_value_ - old_q_value_;
    if(old_behavior_ == 1)
    {
        ROS_INFO_STREAM("updating behavior " << old_behavior_);
        ROS_ERROR_STREAM(" - error " << error);
        ROS_ERROR_STREAM(" - old q value " << old_q_value_);
        ROS_ERROR_STREAM(" - new q value " << new_q_value_);
        ROS_ERROR_STREAM(" - q value " << getQValue(new_game_state, old_behavior_));
    }
    if(old_behavior_ == 3)
    {
        ROS_INFO_STREAM("updating behavior " << old_behavior_);
        ROS_WARN_STREAM(" - error " << error);
        ROS_WARN_STREAM(" - old q value " << old_q_value_);
        ROS_WARN_STREAM(" - new q value " << new_q_value_);
        ROS_WARN_STREAM(" - q value " << getQValue(new_game_state, old_behavior_));
    }

    //std::vector<double>::iterator weights_it = weights_.begin();

    std::vector<double> weights = behavioral_weights_[old_behavior_];
    std::vector<double>::iterator weights_it = weights.begin();

    std::vector<double>::iterator features_it = old_features_.begin();

    for(int i = 0; features_it != old_features_.end() ; ++i, ++features_it, ++weights_it)
    {
        *weights_it = *weights_it + learning_rate_ * error * *features_it;
        /*if(old_behavior_ == 1)
        {
            ROS_ERROR_STREAM(" - feature " << *features_it);
            ROS_ERROR_STREAM(" - weight " << *weights_it);
        }
        if(old_behavior_ == 3)
        {
            ROS_WARN_STREAM(" - feature " << *features_it);
            ROS_WARN_STREAM(" - weight " << *weights_it);
        }*/
    }
    behavioral_weights_[old_behavior_] = weights;


    // TODO: remove after this
    if(old_behavior_ == 1 || old_behavior_ == 3)
    {
        weights = behavioral_weights_[1];
        weights_it = weights.begin();

        for(int i = 0; weights_it != weights.end() ; ++weights_it)
        {
            ROS_ERROR_STREAM(" - 1 weight " << *weights_it);
        }
        weights = behavioral_weights_[3];
        weights_it = weights.begin();

        for(int i = 0; weights_it != weights.end() ; ++weights_it)
        {
            ROS_WARN_STREAM(" - 3 weight " << *weights_it);
        }
    }

    if(old_behavior_ == 1)
    {
        ROS_ERROR_STREAM(" - new q value " << getQValue(new_game_state, old_behavior_));
    }
    if(old_behavior_ == 3)
    {
        ROS_WARN_STREAM(" - new q value " << getQValue(new_game_state, old_behavior_));
    }
}

// TODO: features ideas: closest food with min probability_of_close_enemy
// TODO:                 probability of enemy one step away
// TODO:                 weighted ghost distance, based on probabilities