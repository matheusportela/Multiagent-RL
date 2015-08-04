#include "q_learning_pacman/q_learning.h"
#include "pacman_abstract_classes/util_functions.h"

int QLearning::NUM_BEHAVIORS = 5;
int QLearning::NUM_FEATURES = 7;
double QLearning::learning_rate_ = 0.5;
double QLearning::discount_factor_ = 0.95;
double QLearning::exploration_rate_ = 0.5;
int QLearning::num_training_ = 10;

QLearning::QLearning()
{
    features_= std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    old_features_= std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    weights_ = std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    old_q_value_ = 0;
    new_q_value_ = 0;
    old_behavior_ = 0;
    behavior_ = 0;
}

std::vector<double> QLearning::getFeatures(BayesianGameState *game_state)
{

    for(int i = 0; i < NUM_BEHAVIORS ; ++i)
    {
        features_[i] = 0;
    }

    std::vector< std::vector<float> > pacman_pose = game_state->getPacmanPoseMap();
    std::vector< std::vector< std::vector<float> > > ghosts_poses = game_state->getGhostsPoseMaps();
    std::vector< std::vector<float> > foods = game_state->getFoodMap();

    double max_probability = -util::INFINITE;
    int pacman_x = 0;
    int pacman_y = 0;

    // get most probable pacman position
    for(int i = pacman_pose.size() - 1; i > -1  ; --i)
    {
        for(int j = pacman_pose[i].size() - 1; j > -1 ; --j)
        {
            if(max_probability < pacman_pose[i][j])
            {
                max_probability = pacman_pose[i][j];
                pacman_x = j;
                pacman_y = i;
            }
        }
    }

    // get closest of most probable ghost distances
    int num_ghosts = ghosts_poses.size();
    std::vector<int> closest_distances = std::vector<int> (num_ghosts, util::INFINITE);

    for(int ghost_counter = num_ghosts - 1; ghost_counter > -1  ; --ghost_counter)
    {
        std::vector< std::vector<float> > ghost_pose = ghosts_poses[ghost_counter];
        int distance = -1;
        max_probability = -util::INFINITE;
        for(int i = ghost_pose.size() - 1; i > -1  ; --i)
        {
            for(int j = ghost_pose[i].size() - 1; j > -1 ; --j)
            {
                if(max_probability < ghost_pose[i][j])
                {
                    max_probability = ghost_pose[i][j];
                    distance = ( i - pacman_y ) * ( i - pacman_y ) + ( j - pacman_x ) * ( j - pacman_x );
                }
            }
        }
        closest_distances.push_back(distance);
    }

    int closest_distance = util::INFINITE;
    for(std::vector<int>::reverse_iterator it = closest_distances.rbegin() ; it != closest_distances.rend() ; ++it)
    {
        if (*it < closest_distance)
        {
            closest_distance = *it;
        }
    }

    //ROS_INFO_STREAM("closest distance is " << closest_distance);

    if(closest_distance == 0)
    {
        features_[NUM_BEHAVIORS + 0] = 1.0;
    }
    else
    {
        features_[NUM_BEHAVIORS + 0] = 1.0/closest_distance;
    }

    return features_;
}

double QLearning::getQValue(BayesianGameState *game_state, int behavior)
{
    double q_value = 0;

    features_ = getFeatures(game_state);
    std::vector<double>::iterator features_it = features_.begin();
    std::vector<double>::iterator weights_it = weights_.begin();
    for(int i = 0; features_it != features_.end() && weights_it != weights_.end() ; ++i, ++features_it, ++weights_it)
    {
        if(i == behavior)
        {
            q_value += 1 * *weights_it;
        }
        else
        {
            q_value += *features_it * *weights_it;
        }
        //ROS_INFO_STREAM(" - - - q value " << q_value << " features_ " << *features_it << " weight " << *weights_it);
    }
    //ROS_INFO_STREAM(" - - q value " << q_value << " for behavior " << behavior);

    return q_value;
}

std::pair<int, double> QLearning::getMaxQValue(BayesianGameState *game_state)
{
    int behavior = -1;
    double q_value = 0;
    double max_q_value = - util::INFINITE;

    for(int i = 0; i < NUM_BEHAVIORS ; ++i)
    {
        q_value = getQValue(game_state, i);
    //    ROS_INFO_STREAM(" - - q value " << q_value);
        if(q_value > max_q_value)
        {
            max_q_value = q_value;
            behavior = i;
        }
    }

    return std::make_pair(behavior, max_q_value);
}

int QLearning::getBehavior(BayesianGameState *game_state)
{
    std::pair<int, double> behavior_q_value_pair = getMaxQValue(game_state);
    int behavior = behavior_q_value_pair.first;
    old_q_value_ = behavior_q_value_pair.second;
    return behavior;
}

void QLearning::updateWeights(BayesianGameState *new_game_state, int reward)
{
    old_features_ = features_;
    std::pair<int, double> behavior_q_value_pair = getMaxQValue(new_game_state);

    // error = reward + discount_factor * q_value(new_state) - q_value(old_state)
    double error = reward + discount_factor_ * new_q_value_ - old_q_value_;
    //ROS_INFO_STREAM(" - error " << error);

    std::vector<double>::iterator weights_it = weights_.begin();
    std::vector<double>::iterator features_it = old_features_.begin();

    for(int i = 0; features_it != old_features_.end() ; ++i, ++features_it, ++weights_it)
    {
        *weights_it = *weights_it + learning_rate_ * error * *features_it;
    //    ROS_INFO_STREAM(" - feature " << *features_it);
    //    ROS_INFO_STREAM(" - weight " << *weights_it);
    }
}

// TODO: features ideas: closest food with min probability_of_close_enemy
// TODO:                 probability of enemy one step away
// TODO:                 weighted ghost distance, based on probabilities