#include "simple_q_learning/simple_q_learning.h"
#include "pacman_abstract_classes/util_functions.h"

int SimpleQLearning::NUM_BEHAVIORS = 5;
int SimpleQLearning::NUM_FEATURES = 7;
double SimpleQLearning::learning_rate_ = 0.2;
double SimpleQLearning::discount_factor_ = 0.8;
double SimpleQLearning::exploration_rate_ = 0.05;
int SimpleQLearning::num_training_ = 10;

SimpleQLearning::SimpleQLearning()
{
    //features_= std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    //old_features_= std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    weights_ = std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    old_q_value_ = 0;
    new_q_value_ = 0;
    old_behavior_ = 0;
    behavior_ = 0;
}

void SimpleQLearning::saveTempFeatures()
{
    features_ = temp_features_;
}

std::vector<double> SimpleQLearning::getFeatures(DeterministicGameState *game_state, int behavior)
{
    std::vector<double> features;

    // TODO: get features
    features.push_back(1.0); // bias

    pacman_msgs::PacmanAction action;
    action.action = behavior;

    bool is_legal = game_state->isActionLegal(action);

    // get distances can't be manhattan
    features.push_back( game_state->eatsFood(action) / 10.0 );
    features.push_back( game_state->getClosestFoodDistance(action) / 10.0 );
    features.push_back( game_state->getNumberOfGhostsOneStepAway(action) / 10.0 );
    features.push_back( game_state->getClosestGhostDistance(action) / 10.0 );
    features.push_back( game_state->dies(action) );

    return features;
}

double SimpleQLearning::getQValue(DeterministicGameState *game_state, int behavior)
{
    bool is_legal = game_state->isActionLegal(behavior);
    if(!is_legal)
        return -util::INFINITE;

    double q_value = 0;

    temp_features_ = getFeatures(game_state, behavior);
    std::vector<double>::iterator features_it = temp_features_.begin();
    std::vector<double>::iterator weights_it = weights_.begin();
    for(int i = 0; features_it != temp_features_.end() && weights_it != weights_.end() ; ++i, ++features_it, ++weights_it)
    {
        q_value += *features_it * *weights_it;
        //ROS_INFO_STREAM(" - - - " << i << " q value " << q_value << " features_ " << *features_it << " weight " << *weights_it);
    }
    //ROS_INFO_STREAM(" - - q value " << q_value << " for behavior " << behavior);

    return q_value;
}

std::pair<int, double> SimpleQLearning::getMaxQValue(DeterministicGameState *game_state)
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
            saveTempFeatures();
        }
    }

    return std::make_pair(behavior, max_q_value);
}

int SimpleQLearning::getBehavior(DeterministicGameState *game_state)
{
    std::pair<int, double> behavior_q_value_pair = getMaxQValue(game_state);
    int behavior = behavior_q_value_pair.first;
    old_q_value_ = behavior_q_value_pair.second;
    return behavior;
}

int SimpleQLearning::getTrainingBehavior(DeterministicGameState *game_state)
{
    double random = rand() / (float) RAND_MAX;
    if (random < exploration_rate_) {
        std::vector< pacman_msgs::PacmanAction > legalActions = game_state->getLegalActions();

        int randomAction = rand() % legalActions.size();
        int behavior = legalActions[randomAction].action;

        old_q_value_ = getQValue(game_state, behavior);
        saveTempFeatures();

        return behavior;
    }
    else
        return getBehavior(game_state);
}

void SimpleQLearning::updateWeights(DeterministicGameState *new_game_state, int reward)
{
    old_features_ = features_;
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
    /*ROS_INFO_STREAM(" - reward " << reward);
    ROS_INFO_STREAM(" - old q value " << old_q_value_);
    ROS_INFO_STREAM(" - new q value " << new_q_value_);
    ROS_INFO_STREAM(" - error " << error);*/

    std::vector<double>::iterator weights_it = weights_.begin();
    std::vector<double>::iterator features_it = old_features_.begin();

    for(int i = 0; features_it != old_features_.end() ; ++i, ++features_it, ++weights_it)
    {
        *weights_it = *weights_it + learning_rate_ * error * *features_it;
        //ROS_INFO_STREAM(" - feature " << *features_it);
        //ROS_INFO_STREAM(" - weight " << *weights_it);
    }
}

// TODO: features ideas: closest food with min probability_of_close_enemy
// TODO:                 probability of enemy one step away
// TODO:                 weighted ghost distance, based on probabilities