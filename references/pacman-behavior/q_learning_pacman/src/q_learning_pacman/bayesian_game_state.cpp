#include "q_learning_pacman/bayesian_game_state.h"

#include "pacman_abstract_classes/util_functions.h"


BayesianGameState::BayesianGameState()
{
    //pacman_pose_subscriber_ = n_.subscribe<geometry_msgs::Pose>
                    ("/pacman/pacman_pose", 1000, boost::bind(&BayesianGameState::updatePacman, this, _1));
    //ghost_distance_subscriber_ = n_.subscribe<pacman_msgs::AgentPose>
                    ("/pacman/ghost_distance", 1000, boost::bind(&BayesianGameState::updateGhosts, this, _1));

    pacman_observer_service_ = n_.advertiseService<pacman_msgs::AgentPoseService::Request, pacman_msgs::AgentPoseService::Response>
                                ("/pacman/pacman_pose", boost::bind(&BayesianGameState::observeAgent, this, _1, _2));
    ghost_distance_observer_service_ = n_.advertiseService<pacman_msgs::AgentPoseService::Request, pacman_msgs::AgentPoseService::Response>
                                ("/pacman/ghost_distance", boost::bind(&BayesianGameState::observeAgent, this, _1, _2));

    ROS_DEBUG_STREAM("Bayesian game state initialized");
}

BayesianGameState::~BayesianGameState()
{
        pacman_observer_service_.shutdown();
        ghost_distance_observer_service_.shutdown();

        ROS_DEBUG_STREAM("Bayesian game state destroyed");
}

void BayesianGameState::updateGhosts(const pacman_msgs::AgentPose::ConstPtr& msg)
{
    //ROS_INFO_STREAM("Observe Ghost");
    int ghost_index = msg->agent - 1;
    if(ghost_index == -1)
    {
        ROS_ERROR("Error, trying to update ghost agent with pacman index");
        return;
    }
    int measurement_x = msg->pose.position.x;
    int measurement_y = msg->pose.position.y;
    observeGhost(measurement_x, measurement_y, ghost_index);
}

void BayesianGameState::updatePacman(const geometry_msgs::Pose::ConstPtr& msg)
{
    //ROS_INFO_STREAM("Observe Pacman");
    int measurement_x = msg->position.x;
    int measurement_y = msg->position.y;
    observePacman(measurement_x, measurement_y);
}

void BayesianGameState::observePacman(int measurement_x, int measurement_y)
{
    //ROS_INFO_STREAM("pos x " << measurement_x << " y " << measurement_y);
    //printPacmanOrGhostPose(true, 0);

    float SD_PACMAN_MEASUREMENT = 0.01;

    std::vector<float> pacman_pose_map_line (width_, 0);
    std::vector< std::vector<float> > pacman_new_pose_map (height_, pacman_pose_map_line);
    float sum_probabilities = 0.0;

    for (int i = 0 ; i < width_ ; i++)
    {
        for (int j = 0 ; j < height_ ; j++)
        {
            if( map_[j][i] != WALL)
            {
                float old_probability_of_being_in_this_place = pacman_pose_map_[j][i];
                float probability_of_place_given_z = util::getProbOfMeasurementGivenPosition(i, j, measurement_x, measurement_y, SD_PACMAN_MEASUREMENT);

                //ROS_INFO_STREAM("" << random_probability);
                pacman_new_pose_map[j][i] = probability_of_place_given_z * old_probability_of_being_in_this_place;

                sum_probabilities += pacman_new_pose_map[j][i];
            }
        }
    }

    for (int i = 0 ; i < width_ ; i++)
    {
        for (int j = 0 ; j < height_ ; j++)
        {
            if( map_[j][i] != WALL)
            {
                if(sum_probabilities != 0)
                    pacman_new_pose_map[j][i] = pacman_new_pose_map[j][i]/sum_probabilities;
                else
                {
                    ROS_WARN_STREAM_THROTTLE(1, "Probability 0 for pacman, redistributing");
                    pacman_new_pose_map[j][i] = 1.0 / (float) ( height_ * width_ );
                }
            }
        }
    }

    pacman_pose_map_.clear();
    pacman_pose_map_ = pacman_new_pose_map;

    //printPacmanOrGhostPose(true, 0);
}

void BayesianGameState::observeGhost(int measurement_x_dist, int measurement_y_dist, int ghost_index)
{
    /*if(ghost_index == 1) {
        ROS_INFO_STREAM("pos x " << measurement_x_dist << " y " << measurement_y_dist);
        printPacmanOrGhostPose(false, ghost_index);
    }*/
    double SD_GHOST_DIST_MEASUREMENT = 0.01;

    std::vector< std::vector<float> > ghost_pose_map = ghosts_poses_map_[ghost_index];

    std::vector<float> ghost_pose_map_line (width_, 0);
    std::vector< std::vector<float> > ghost_new_pose_map (height_, ghost_pose_map_line);
    float sum_probabilities = 0.0;

    for (int i = 0 ; i < width_ ; i++)
    {
        for (int j = 0 ; j < height_ ; j++)
        {
            if( map_[j][i] != WALL)
            {
                float old_probability_of_being_in_this_place = ghost_pose_map[j][i];
                for (int pacman_i = 0 ; pacman_i < width_ ; pacman_i++)
                {
                    for (int pacman_j = 0 ; pacman_j < height_ ; pacman_j++)
                    {
                        float probability_of_pacman = pacman_pose_map_[pacman_j][pacman_i];
                        float probability_of_place_given_z = util::getProbOfMeasurementGivenPosition(i - pacman_i, j - pacman_j, measurement_x_dist, measurement_y_dist, SD_GHOST_DIST_MEASUREMENT);

                        //ROS_INFO_STREAM("" << random_probability);
                        ghost_new_pose_map[j][i] += probability_of_place_given_z * old_probability_of_being_in_this_place * probability_of_pacman;
                    }
                }
                sum_probabilities += ghost_new_pose_map[j][i];
            }
        }
    }

    if(sum_probabilities == 0)
    {
        ROS_WARN_STREAM("Probability 0 for ghost " << ghost_index << ", redistributing");
    }

    for (int i = 0 ; i < width_ ; i++)
    {
        for (int j = 0 ; j < height_ ; j++)
        {
            if( map_[j][i] != WALL)
            {
                if(sum_probabilities != 0)
                    ghost_new_pose_map[j][i] = ghost_new_pose_map[j][i]/sum_probabilities;
                else
                    ghost_new_pose_map[j][i] = 1.0 / (float) ( height_ * width_ );
            }
        }
    }

    ghosts_poses_map_[ghost_index].clear();
    ghosts_poses_map_[ghost_index] = ghost_new_pose_map;

    //if(ghost_index == 1)
    //    printPacmanOrGhostPose(false, ghost_index);
}

bool BayesianGameState::observeAgent(pacman_msgs::AgentPoseService::Request &req, pacman_msgs::AgentPoseService::Response &res)
{
    int agent = (int) req.agent;
    geometry_msgs::Pose pose = (geometry_msgs::Pose) req.pose;
    int measurement_x = pose.position.x;
    int measurement_y = pose.position.y;

    if(agent == pacman_msgs::AgentPoseService::Request::PACMAN)
    {
        ROS_INFO_STREAM("Observe pacman");
        observePacman(measurement_x, measurement_y);
    }
    else
    {
        int ghost_index = agent - 1;
        ROS_INFO_STREAM("Observe ghost " << ghost_index);
        observeGhost(measurement_x, measurement_y, ghost_index);
    }

    res.observed = true;
    return true;
}

void BayesianGameState::predictPacmanMove(pacman_msgs::PacmanAction action)
{
    ROS_INFO_STREAM("Predict pacman");

    std::vector<float> pacman_pose_map_line (width_, 0);
    std::vector< std::vector<float> > pacman_new_pose_map (height_, pacman_pose_map_line);
    pacman_pose_map_line.clear();

    for (int i = 0 ; i < width_ ; i++)
    {
        for (int j = 0 ; j < height_ ; j++)
        {
            if( map_[j][i] != WALL)
            {
                float probability_of_being_in_this_place = pacman_pose_map_[j][i];

                std::vector< std::pair< float, std::pair<int, int> > > next_positions = getNextPositionsForActionWithProbabilities(i, j, action);

                //ROS_INFO_STREAM("" << random_probability);

                for(std::vector< std::pair< float, std::pair<int, int> > >::reverse_iterator it = next_positions.rbegin(); it != next_positions.rend(); ++it)
                {
                    /* std::cout << *it; ... */
                    float probability_of_move = it->first;
                    int x = it->second.first;
                    int y = it->second.second;
                    pacman_new_pose_map[y][x] += probability_of_move * probability_of_being_in_this_place;
                }
            }
        }
    }

    pacman_pose_map_.clear();
    pacman_pose_map_ = pacman_new_pose_map;
}

void BayesianGameState::predictGhostMove(int ghost_index)
{
    ROS_INFO_STREAM("Predict ghost " << ghost_index);

    double STOP_PROBABILITY = 0.2;

    std::vector<float> ghost_pose_map_line (width_, 0);
    std::vector< std::vector<float> > ghost_new_pose_map (height_, ghost_pose_map_line);

    std::vector< std::vector<float> > ghost_pose_map = ghosts_poses_map_[ghost_index];

    for (int i = 0 ; i < width_ ; i++)
    {
        for (int j = 0 ; j < height_ ; j++)
        {
            if( map_[j][i] != WALL)
            {
                float probability_of_being_in_this_place = ghost_pose_map[j][i];

                std::vector< std::pair<int, int> > next_positions = getLegalNextPositions(i, j);
                float random_probability = (1.0 - STOP_PROBABILITY)/next_positions.size();

                ghost_new_pose_map[j][i] += STOP_PROBABILITY * probability_of_being_in_this_place;

                //ROS_INFO_STREAM("" << random_probability);

                for(std::vector< std::pair<int, int> >::reverse_iterator it = next_positions.rbegin(); it != next_positions.rend(); ++it)
                {
                    /* std::cout << *it; ... */
                    int x = it->first;
                    int y = it->second;
                    ghost_new_pose_map[y][x] += random_probability * probability_of_being_in_this_place;
                }
            }
        }
    }

    ghosts_poses_map_[ghost_index].clear();
    ghosts_poses_map_[ghost_index] = ghost_new_pose_map;
}

void BayesianGameState::predictGhostsMoves()
{
    for(int i = 0 ; i < num_ghosts_ ; ++i)
    {
        predictGhostMove(i);
    }
}

void BayesianGameState::predictAgentsMoves(pacman_msgs::PacmanAction action)
{
    predictPacmanMove(action);
    predictGhostsMoves();
}