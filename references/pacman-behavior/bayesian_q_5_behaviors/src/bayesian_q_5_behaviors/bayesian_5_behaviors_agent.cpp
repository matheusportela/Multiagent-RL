#include "bayesian_q_5_behaviors/bayesian_5_behaviors_agent.h"

int BayesianBehaviorAgent::NUMBER_OF_BEHAVIORS_ = 5;

BayesianBehaviorAgent::BayesianBehaviorAgent()
{
    ROS_DEBUG("Behavior Agent initialized");
}

pacman_msgs::PacmanAction BayesianBehaviorAgent::getAction(BayesianGameState *game_state, int behavior)
{
    pacman_msgs::PacmanAction action;

    switch (behavior)
    {
        case STOP:
            //ROS_INFO_STREAM("Stop behavior");
            action = getStopAction();
            break;
        case EAT:
            //ROS_INFO_STREAM("Eat behavior");
            action = getEatAction(game_state);
            break;
        case EAT_BIG_FOOD:
            //ROS_INFO_STREAM("Eat big food behavior");
            action = getEatBigFoodAction(game_state);
            break;
        case RUN:
            //ROS_INFO_STREAM("Run behavior");
            action = getRunAction(game_state);
            break;
        case HUNT:
            //ROS_INFO_STREAM("Hunt behavior");
            action = getHuntAction(game_state);
            break;
        default:
            ROS_ERROR_STREAM("Unknown default behavior");
            action.action = action.STOP;
            break;
    }

    return action;
}

std::string BayesianBehaviorAgent::getAgentName()
{
    return "BayesianBehaviorAgent";
}

pacman_msgs::PacmanAction BayesianBehaviorAgent::getHuntAction(BayesianGameState *game_state) {
    geometry_msgs::Pose pacman_pose = game_state->getMostProbablePacmanPose();
    std::map< std::pair<int, int>, int > distances = game_state->getDistances(pacman_pose.position.x, pacman_pose.position.y);

    int min_distance = util::MAX_DISTANCE;

    std::vector< geometry_msgs::Pose > ghosts_poses = game_state->getMostProbableGhostsPoses();
    std::vector< geometry_msgs::Pose >::reverse_iterator closest_ghost;
    bool found_ghost = false;
    for(std::vector< geometry_msgs::Pose >::reverse_iterator it = ghosts_poses.rbegin(); it != ghosts_poses.rend(); ++it) {
        /* std::cout << *it; ... */
        int distance = distances[std::make_pair(it->position.x, it->position.y)];
        if(distance != 0 && distance < min_distance)
        {
            closest_ghost = it;
            min_distance = distance;
            found_ghost = true;
        }
    }

    if(!found_ghost)
        closest_ghost = ghosts_poses.rbegin();

    distances = game_state->getDistances(closest_ghost->position.x, closest_ghost->position.y);
    std::vector< pacman_msgs::PacmanAction > actions = game_state->getLegalActions(pacman_pose.position.x, pacman_pose.position.y);
    std::vector< std::pair<int, int> > next_positions = game_state->getLegalNextPositions(pacman_pose.position.x, pacman_pose.position.y);

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

pacman_msgs::PacmanAction BayesianBehaviorAgent::getRunAction(BayesianGameState *game_state) {
    pacman_msgs::PacmanAction action;

    geometry_msgs::Pose pacman_pose = game_state->getMostProbablePacmanPose();
    std::map< std::pair<int, int>, int > distances = game_state->getDistances(pacman_pose.position.x, pacman_pose.position.y);

    int min_distance = util::MAX_DISTANCE;

    std::vector< geometry_msgs::Pose > ghosts_poses = game_state->getMostProbableGhostsPoses();
    std::vector< geometry_msgs::Pose >::reverse_iterator closest_ghost;
    bool found_ghost = false;
    for(std::vector< geometry_msgs::Pose >::reverse_iterator it = ghosts_poses.rbegin(); it != ghosts_poses.rend(); ++it) {
        /* std::cout << *it; ... */
        int distance = distances[std::make_pair(it->position.x, it->position.y)];
        if(distance != 0 && distance < min_distance)
        {
            closest_ghost = it;
            min_distance = distance;
            found_ghost = true;
        }
    }

    if(!found_ghost)
        closest_ghost = ghosts_poses.rbegin();

    distances = game_state->getDistances(closest_ghost->position.x, closest_ghost->position.y);
    std::vector< pacman_msgs::PacmanAction > actions = game_state->getLegalActions(pacman_pose.position.x, pacman_pose.position.y);
    std::vector< std::pair<int, int> > next_positions = game_state->getLegalNextPositions(pacman_pose.position.x, pacman_pose.position.y);

    int action_iterator = 0;
    for(int i = next_positions.size() - 1; i != -1; i--)
    {
        if ( distances[ next_positions[i] ] > min_distance )
        {
            action_iterator = i;
            break;
        }
    }

    return actions[action_iterator];
}

pacman_msgs::PacmanAction BayesianBehaviorAgent::getEatBigFoodAction(BayesianGameState *game_state) {
    geometry_msgs::Pose pacman_pose = game_state->getMostProbablePacmanPose();
    std::map< std::pair<int, int>, int > distances = game_state->getDistances(pacman_pose.position.x, pacman_pose.position.y);
    std::vector< std::vector<float> > big_foods_map = game_state->getBigFoodMap();
    float food_probability_threshold = game_state->getMaxBigFoodProbability()/2.0;

    int width = game_state->getWidth();
    int height = game_state->getHeight();

    int min_distance = util::MAX_DISTANCE;
    int food_x = -1;
    int food_y = -1;
    for (int i = 0 ; i < width ; i++)
    {
        for (int j = 0 ; j < height ; j++)
        {
            if( big_foods_map[j][i] >= food_probability_threshold)
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

    distances = game_state->getDistances(food_x, food_y);
    std::vector< pacman_msgs::PacmanAction > actions = game_state->getLegalActions(pacman_pose.position.x, pacman_pose.position.y);
    std::vector< std::pair<int, int> > next_positions = game_state->getLegalNextPositions(pacman_pose.position.x, pacman_pose.position.y);

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

pacman_msgs::PacmanAction BayesianBehaviorAgent::getEatAction(BayesianGameState *game_state) {
    pacman_msgs::PacmanAction action;

    geometry_msgs::Pose pacman_pose = game_state->getMostProbablePacmanPose();
    std::map< std::pair<int, int>, int > distances = game_state->getDistances(pacman_pose.position.x, pacman_pose.position.y);
    std::vector< std::vector<float> > foods_map = game_state->getFoodMap();
    float food_probability_threshold = game_state->getMaxFoodProbability()/2.0;

    int width = game_state->getWidth();
    int height = game_state->getHeight();

    int min_distance = util::MAX_DISTANCE;
    int food_x = -1;
    int food_y = -1;
    for (int i = 0 ; i < width ; i++)
    {
        for (int j = 0 ; j < height ; j++)
        {
            if( foods_map[j][i] >= food_probability_threshold)
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

    distances = game_state->getDistances(food_x, food_y);
    std::vector< pacman_msgs::PacmanAction > actions = game_state->getLegalActions(pacman_pose.position.x, pacman_pose.position.y);
    std::vector< std::pair<int, int> > next_positions = game_state->getLegalNextPositions(pacman_pose.position.x, pacman_pose.position.y);

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

pacman_msgs::PacmanAction BayesianBehaviorAgent::getStopAction() {
    pacman_msgs::PacmanAction action;
    action.action = pacman_msgs::PacmanAction::STOP;
    return action;
}