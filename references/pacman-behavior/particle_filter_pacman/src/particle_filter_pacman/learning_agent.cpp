#include "particle_filter_pacman/learning_agent.h"

#include "ros/ros.h"
#include "particle_filter_pacman/util_constants.h"

#include "pacman_interface/PacmanAction.h"
#include "std_msgs/String.h"

#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>


LearningAgent::LearningAgent()
{
    action_publisher_ = n_.advertise<pacman_interface::PacmanAction>("/pacman_interface/pacman_action", 1000);

    ROS_DEBUG("Learning Agent initialized");
}

pacman_interface::PacmanAction LearningAgent::sendAction(ParticleFilter *particle_filter, int behavior)
{
    pacman_interface::PacmanAction action;

    switch (behavior)
    {
        case STOP:
            action = getStopAction();
            ROS_INFO_STREAM("Stop behavior");
            break;
        case EAT:
            action = getEatAction(particle_filter);
            ROS_INFO_STREAM("Eat behavior");
            break;
        case EAT_BIG_FOOD:
            action = getEatBigFoodAction(particle_filter);
            ROS_INFO_STREAM("Eat big food behavior");
            break;
        case RUN:
            action = getRunAction(particle_filter);
            ROS_INFO_STREAM("Run behavior");
            break;
        case HUNT:
            action = getHuntAction(particle_filter);
            ROS_INFO_STREAM("Hunt behavior");
            break;
        default:
            action.action = action.STOP;
            ROS_ERROR_STREAM("Unknown default behavior");
            break;
    }

    action_publisher_.publish(action);
    particle_filter->resetNewObservation();

    return action;
}

std::string LearningAgent::getAgentName()
{
    return "LearningAgent";
}

pacman_interface::PacmanAction LearningAgent::getHuntAction(ParticleFilter *particle_filter) {
    pacman_interface::PacmanAction action;
    action.action = action.STOP;

    return action;
}

pacman_interface::PacmanAction LearningAgent::getRunAction(ParticleFilter *particle_filter) {
    pacman_interface::PacmanAction action;

    geometry_msgs::Pose pacman_pose = particle_filter->getEstimatedPacmanPose();
    std::map< std::pair<int, int>, int > distances = particle_filter->getDistances(pacman_pose.position.x, pacman_pose.position.y);

    int min_distance = util::MAX_DISTANCE;

    std::vector< geometry_msgs::Pose > ghosts_poses = particle_filter->getEstimatedGhostsPoses();
    std::vector< geometry_msgs::Pose >::reverse_iterator closest_ghost;
    for(std::vector< geometry_msgs::Pose >::reverse_iterator it = ghosts_poses.rbegin(); it != ghosts_poses.rend(); ++it) {
        int distance = distances[std::make_pair(it->position.x, it->position.y)];
        if(distance != 0 && distance < min_distance)
        {
            closest_ghost = it;
            min_distance = distance;
        }
    }

    ROS_INFO_STREAM("x: " << closest_ghost->position.x << " y: " << closest_ghost->position.y );

    distances = particle_filter->getDistances(closest_ghost->position.x, closest_ghost->position.y);
    std::vector< pacman_interface::PacmanAction > actions = particle_filter->getLegalActions(pacman_pose.position.x, pacman_pose.position.y);
    std::vector< std::pair<int, int> > next_positions = particle_filter->getLegalNextPositions(pacman_pose.position.x, pacman_pose.position.y);

    int action_iterator = -1;
    for(int i = next_positions.size() - 1; i != -1; i--)
    {
        ROS_INFO_STREAM("- distance " << (short) actions[i].action << " : " << distances[ next_positions[i] ] );
        if ( distances[ next_positions[i] ] > min_distance )
        {
            action_iterator = i;
            break;
        }
    }
    if(action_iterator == -1)
    {
        action.action = action.STOP;
    }
    else
    {
        action = actions[action_iterator];
    }

    return action;
}

pacman_interface::PacmanAction LearningAgent::getEatBigFoodAction(ParticleFilter *particle_filter) {
    pacman_interface::PacmanAction action;

    geometry_msgs::Pose pacman_pose = particle_filter->getEstimatedPacmanPose();
    std::map< std::pair<int, int>, int > distances = particle_filter->getDistances(pacman_pose.position.x, pacman_pose.position.y);

    int width = particle_filter->getMapWidth();
    int height = particle_filter->getMapHeight();
    std::vector< std::vector<GameParticle::MapElements> > map = particle_filter->getEstimatedMap();

    int min_distance = util::INFINITE;
    int food_x = -1;
    int food_y = -1;
    for (int i = 0 ; i < width ; i++)
    {
        for (int j = 0 ; j < height ; j++)
        {
            if( map[j][i] == GameParticle::BIG_FOOD)
            {
                int distance = distances[std::make_pair(i, j)];
                if(distance < min_distance)
                {
                    food_x = i;
                    food_y = j;
                    min_distance = distance;
                }
            }
        }
    }

    distances = particle_filter->getDistances(food_x, food_y);

    std::vector< pacman_interface::PacmanAction > actions = particle_filter->getLegalActions(pacman_pose.position.x, pacman_pose.position.y);
    std::vector< std::pair<int, int> > next_positions = particle_filter->getLegalNextPositions(pacman_pose.position.x, pacman_pose.position.y);

    int action_iterator = 0;
    for(int i = next_positions.size() - 1; i != -1; i--)
    {
        if ( distances[ next_positions[i] ] < min_distance )
        {
            action_iterator = i;
            break;
        }
    }

    return actions[action_iterator];
}

pacman_interface::PacmanAction LearningAgent::getEatAction(ParticleFilter *particle_filter) {
    pacman_interface::PacmanAction action;

    geometry_msgs::Pose pacman_pose = particle_filter->getEstimatedPacmanPose();
    std::map< std::pair<int, int>, int > distances = particle_filter->getDistances(pacman_pose.position.x, pacman_pose.position.y);

    int width = particle_filter->getMapWidth();
    int height = particle_filter->getMapHeight();
    std::vector< std::vector<GameParticle::MapElements> > map = particle_filter->getEstimatedMap();

    int min_distance = util::INFINITE;
    int food_x = -1;
    int food_y = -1;
    for (int i = 0 ; i < width ; i++)
    {
        for (int j = 0 ; j < height ; j++)
        {
            if( map[j][i] == GameParticle::FOOD)
            {
                int distance = distances[std::make_pair(i, j)];
                if(distance < min_distance)
                {
                    food_x = i;
                    food_y = j;
                    min_distance = distance;
                }
            }
        }
    }

    distances = particle_filter->getDistances(food_x, food_y);

    std::vector< pacman_interface::PacmanAction > actions = particle_filter->getLegalActions(pacman_pose.position.x, pacman_pose.position.y);
    std::vector< std::pair<int, int> > next_positions = particle_filter->getLegalNextPositions(pacman_pose.position.x, pacman_pose.position.y);

    int action_iterator = 0;
    for(int i = next_positions.size() - 1; i != -1; i--)
    {
        if ( distances[ next_positions[i] ] < min_distance )
        {
            action_iterator = i;
            break;
        }
    }

    return actions[action_iterator];
}

pacman_interface::PacmanAction LearningAgent::getStopAction() {
    pacman_interface::PacmanAction action;
    action.action = action.STOP;
    return action;
}