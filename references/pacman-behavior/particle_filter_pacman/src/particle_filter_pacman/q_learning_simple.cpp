#include "particle_filter_pacman/q_learning_simple.h"

#include "particle_filter_pacman/util_constants.h"
#include "pacman_abstract_classes/util_functions.h"

int QLearning::NUM_FEATURES = 2;

QLearningSimple::QLearningSimple()
{
    features_= std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    old_features_= std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    weights_ = std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    old_q_value_ = 0;
    new_q_value_ = 0;
    old_behavior_ = 0;
    behavior_ = 0;
}

std::vector<double> QLearningSimple::getFeatures(ParticleFilter *particle_filter)
{

    for(int i = 0; i < NUM_BEHAVIORS ; ++i)
    {
        features_[i] = 0;
    }

    geometry_msgs::Pose pacman = particle_filter->getEstimatedPacmanPose();
    std::vector< geometry_msgs::Pose > ghosts = particle_filter->getEstimatedGhostsPoses();
    std::vector< std::vector<GameParticle::MapElements> > map = particle_filter->getEstimatedMap();

    std::map< std::pair<int, int>, int > distances = particle_filter->getDistances(pacman.position.x, pacman.position.y);

    // feature 1 / dist_closest_ghost
    int min_distance = util::INFINITE;

    for(std::vector< geometry_msgs::Pose >::reverse_iterator it = ghosts.rbegin(); it != ghosts.rend(); ++it) {
        int distance = distances[std::make_pair(it->position.x, it->position.y)];
        if(distance != 0 && distance < min_distance)
        {
            min_distance = distance;
        }
    }
    if(min_distance == 0)
    {
        features_[NUM_BEHAVIORS + 0] = 1.0;
    }
    else
    {
        features_[NUM_BEHAVIORS + 0] = 1.0/min_distance;
    }

    // feature 1 / dist_closest_food

    int width = particle_filter->getMapWidth();
    int height = particle_filter->getMapHeight();

    min_distance = util::INFINITE;
    for (int i = 0 ; i < width ; i++)
    {
        for (int j = 0 ; j < height ; j++)
        {
            if( map[j][i] == GameParticle::FOOD)
            {
                int distance = distances[std::make_pair(i, j)];
                if(distance < min_distance)
                {
                    min_distance = distance;
                }
            }
        }
    }
    if(min_distance == 0)
    {
        features_[NUM_BEHAVIORS + 1] = 1.0;
    }
    else
    {
        features_[NUM_BEHAVIORS + 1] = 1.0/min_distance;
    }

    return features_;
}

void QLearningSimple::updateFeatures(ParticleFilter *particle_filter)
{
    features_ = getFeatures(particle_filter);
}

double QLearningSimple::getQValue(int behavior)
{
    double q_value = 0;

    std::vector<double>::iterator features_it = features_.begin();
    std::vector<double>::iterator weights_it = weights_.begin();
    for(int i = 0; features_it != features_.end() && weights_it != weights_.end() ; ++i, ++features_it, ++weights_it)
    {
        if(i == behavior)
        {
            q_value += 1 * *weights_it;
    ROS_INFO_STREAM("behavior " << i << " value " << 1 * *weights_it);
        }
        else
        {
    ROS_INFO_STREAM("others " << i << " value " << *features_it * *weights_it);
            q_value += *features_it * *weights_it;
        }
    }
    ROS_INFO_STREAM(" - - q value " << q_value << " for behavior " << behavior);

    return q_value;
}

int QLearningSimple::getMaxQValue()
{
    int behavior = -1;
    double q_value = 0;
    double max_q_value = - util::INFINITE;

    for(int i = 0; i < NUM_BEHAVIORS ; ++i)
    {
        q_value = getQValue(i);
        ROS_INFO_STREAM(" - - q value " << q_value);
        if(q_value > max_q_value)
        {
            max_q_value = q_value;
            behavior = i;
        }
    }
    
    old_q_value_ = new_q_value_;
    new_q_value_ = max_q_value;
    //features_[behavior_] = 1;
    old_behavior_ = behavior_;
    behavior_ = behavior;

    return max_q_value;
}

int QLearningSimple::getBehavior()
{
    getMaxQValue();
    old_features_ = features_;
    old_features_[behavior_] = 1;
    return behavior_;
}

void QLearningSimple::updateWeights(int reward)
{
    getMaxQValue();

    std::vector<double>::iterator features_it = old_features_.begin();
    std::vector<double>::iterator weights_it = weights_.begin();

    double error = 1; // reward + discount_factor * q_value(new_state) - q_value(old_state) // TODO: add this
    error = reward + discount_factor_ * new_q_value_ - old_q_value_;

// TODO: check if old features is correct
    ROS_INFO_STREAM("Updating weights " << old_features_.size());
    ROS_INFO_STREAM(" - new q " << new_q_value_);
    ROS_INFO_STREAM(" - old q " << old_q_value_);
    ROS_INFO_STREAM(" - error " << error);
    ROS_INFO_STREAM(" - reward " << reward);
    for(int i = 0; features_it != old_features_.end() ; ++i, ++features_it, ++weights_it)
    {
        *weights_it = *weights_it + learning_rate_ * error * *features_it;
        ROS_INFO_STREAM(" - feature " << *features_it);
        ROS_INFO_STREAM(" - weight " << *weights_it);
    }
}