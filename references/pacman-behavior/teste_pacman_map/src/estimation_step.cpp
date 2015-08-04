#include <stdio.h>
#include <math.h>

#include "ros/ros.h"
#include "teste_pacman_map/new_game_info.h"

#include "pacman_interface/AgentAction.h"
#include "pacman_interface/AgentPose.h"
#include "geometry_msgs/Pose.h"

static float SD_PACMAN_MEASUREMENT = 1.0;
static float SD_GHOST_DIST_MEASUREMENT = 1.0;

// TODO: Add food probabilities to observe and predict pacman movement

float getProbOfMeasurementGivenPosition(int pos_x, int pos_y, int measurement_x, int measurement_y, double standard_deviation)
{
    // e^( (-1/2) * ( ( x - mean ) / std_deviation ) ^ 2 )
    // e^( - ( ( ( x - mean_x ) ^ 2 + ( y - mean_y ) ^ 2 ) / ( 2 * std_deviation ) ) ^ 2 )

    float diff_x = measurement_x - pos_x;
    float diff_y = measurement_y - pos_y;

    float exponencial_value = - (diff_x*diff_x + diff_y*diff_y) / (2*standard_deviation*standard_deviation);

    return exp(exponencial_value);
}

void observeGhost(NewGameInfo *game_info, int measurement_x_dist, int measurement_y_dist, int ghost_index)
{
    int width = game_info->getWidth();
    int height = game_info->getHeight();

    std::vector< std::vector<float> > pacman_pose_map = game_info->getPacmanPoseMap();
    std::vector< std::vector<float> > ghost_pose_map = game_info->getGhostPoseMap(ghost_index);

    std::vector<float> ghost_pose_map_line (width, 0);
    std::vector< std::vector<float> > ghost_new_pose_map (height, ghost_pose_map_line);
    float sum_probabilities = 0.0;

    for (int i = 0 ; i < width ; i++)
    {
        for (int j = 0 ; j < height ; j++)
        {
            if( game_info->getMapElement(i, j) != NewGameInfo::WALL)
            {
                float old_probability_of_being_in_this_place = ghost_pose_map[j][i];
                for (int pacman_i = 0 ; pacman_i < width ; pacman_i++)
                {
                    for (int pacman_j = 0 ; pacman_j < height ; pacman_j++)
                    {
                        float probability_of_pacman = pacman_pose_map[pacman_j][pacman_i];
                        float probability_of_place_given_z = getProbOfMeasurementGivenPosition(i - pacman_i, j - pacman_j, measurement_x_dist, measurement_y_dist, SD_GHOST_DIST_MEASUREMENT);

                        //ROS_INFO_STREAM("" << random_probability);
                        ghost_new_pose_map[j][i] += probability_of_place_given_z * old_probability_of_being_in_this_place * probability_of_pacman;
                    }
                }
                sum_probabilities += ghost_new_pose_map[j][i];
            }
        }
    }

    for (int i = 0 ; i < width ; i++)
    {
        for (int j = 0 ; j < height ; j++)
        {
            if( game_info->getMapElement(i, j) != NewGameInfo::WALL)
            {
                if(sum_probabilities != 0)
                    ghost_new_pose_map[j][i] = ghost_new_pose_map[j][i]/sum_probabilities;
                else
                {
                    ghost_new_pose_map[j][i] = 1.0 / (float) ( height * width );
                }
            }
        }
    }

    ROS_WARN_STREAM_COND(sum_probabilities == 0, "Probability 0 for ghost " << ghost_index << ", redistributing");
    game_info->setGhostPoseMap(ghost_new_pose_map, ghost_index);
}

void observePacman(NewGameInfo *game_info, int measurement_x, int measurement_y)
{
    ROS_INFO_STREAM("Observe");
    int width = game_info->getWidth();
    int height = game_info->getHeight();

    std::vector< std::vector<float> > pacman_pose_map = game_info->getPacmanPoseMap();

    std::vector<float> pacman_pose_map_line (width, 0);
    std::vector< std::vector<float> > pacman_new_pose_map (height, pacman_pose_map_line);
    float sum_probabilities = 0.0;

    for (int i = 0 ; i < width ; i++)
    {
        for (int j = 0 ; j < height ; j++)
        {
            if( game_info->getMapElement(i, j) != NewGameInfo::WALL)
            {
                float old_probability_of_being_in_this_place = pacman_pose_map[j][i];
                float probability_of_place_given_z = getProbOfMeasurementGivenPosition(i, j, measurement_x, measurement_y, SD_PACMAN_MEASUREMENT);

                //ROS_INFO_STREAM("" << random_probability);
                pacman_new_pose_map[j][i] = probability_of_place_given_z * old_probability_of_being_in_this_place;

                sum_probabilities += pacman_new_pose_map[j][i];
            }
        }
    }

    for (int i = 0 ; i < width ; i++)
    {
        for (int j = 0 ; j < height ; j++)
        {
            if( game_info->getMapElement(i, j) != NewGameInfo::WALL)
            {
                if(sum_probabilities != 0)
                    pacman_new_pose_map[j][i] = pacman_new_pose_map[j][i]/sum_probabilities;
                else
                {
                    ROS_WARN_STREAM_THROTTLE(1, "Probability 0 for pacman, redistributing");
                    pacman_new_pose_map[j][i] = 1.0 / (float) ( height * width );
                }
            }
        }
    }

    game_info->setPacmanPoseMap(pacman_new_pose_map);
    
    game_info->printPacmanOrGhostPose(true, 0);
}

void predictGhostMove(NewGameInfo *game_info, int ghost_index)
{
    int width = game_info->getWidth();
    int height = game_info->getHeight();

    std::vector<float> single_ghost_pose_map_line (width, 0);
    std::vector< std::vector<float> > single_ghost_new_pose_map (height, single_ghost_pose_map_line);

    std::vector< std::vector<float> > single_ghost_pose_map = game_info->getGhostPoseMap(ghost_index);

    for (int i = 0 ; i < width ; i++)
    {
        for (int j = 0 ; j < height ; j++)
        {
            if( game_info->getMapElement(i, j) != NewGameInfo::WALL)
            {
                float probability_of_being_in_this_place = single_ghost_pose_map[j][i];

                std::vector< std::pair<int, int> > next_positions = game_info->getLegalNextPositions(i, j);
                float random_probability = 1.0/next_positions.size();

                //ROS_INFO_STREAM("" << random_probability);

                for(std::vector< std::pair<int, int> >::reverse_iterator it = next_positions.rbegin(); it != next_positions.rend(); ++it)
                {
                    /* std::cout << *it; ... */
                    int x = it->first;
                    int y = it->second;
                    single_ghost_new_pose_map[y][x] += random_probability * probability_of_being_in_this_place;
                }
            }
        }
    }
    game_info->setGhostPoseMap(single_ghost_new_pose_map, ghost_index);
}

void predictPacmanMove(NewGameInfo *game_info, pacman_interface::PacmanAction action)
{
    ROS_INFO_STREAM("Predict " << (int) action.action);
    int width = game_info->getWidth();
    int height = game_info->getHeight();

    std::vector< std::vector<float> > pacman_pose_map = game_info->getPacmanPoseMap();

    std::vector<float> pacman_pose_map_line (width, 0);
    std::vector< std::vector<float> > pacman_new_pose_map (height, pacman_pose_map_line);

    for (int i = 0 ; i < width ; i++)
    {
        for (int j = 0 ; j < height ; j++)
        {
            if( game_info->getMapElement(i, j) != NewGameInfo::WALL)
            {
                float probability_of_being_in_this_place = pacman_pose_map[j][i];

                std::vector< std::pair< float, std::pair<int, int> > > next_positions = game_info->getNextPositionsForActionWithProbabilities(i, j, action);

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
    game_info->setPacmanPoseMap(pacman_new_pose_map);

    game_info->printPacmanOrGhostPose(true, 0);
}

void updateGhosts(const pacman_interface::AgentPose::ConstPtr& msg, NewGameInfo *game_info)
{
    int ghost_index = msg->agent - 1;
    if(ghost_index == -1)
        ROS_ERROR("ERROR");
    int measurement_x = msg->pose.position.x;
    int measurement_y = msg->pose.position.y;
    observeGhost(game_info, measurement_x, measurement_y, ghost_index);
}

void updatePacman(const geometry_msgs::Pose::ConstPtr& msg, NewGameInfo *game_info)
{
    int measurement_x = msg->position.x;
    int measurement_y = msg->position.y;
    observePacman(game_info, measurement_x, measurement_y);
}

void updateAgents(const pacman_interface::AgentAction::ConstPtr& msg, NewGameInfo *game_info)
{

    if(msg->agent == msg->PACMAN)
    {
        pacman_interface::PacmanAction action;
        action.action = (int) msg->action;

        predictPacmanMove(game_info, action);
    }
    else
    {
        int ghost_index = ( (short) msg->agent ) - 1;
        predictGhostMove(game_info, ghost_index);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "estimation_step");
    ros::NodeHandle n;
    ros::Rate loop_rate(5);

    NewGameInfo game_info;
    game_info.printMap();
    game_info.printMap2();

    game_info.printPacmanOrGhostPose( true, 0);
//    game_info.printPacmanOrGhostPose( false, 0);

    ros::Subscriber estimate_action_subscriber = n.subscribe<pacman_interface::AgentAction>
                        ("/pacman_interface/agent_action", 1000, boost::bind(&updateAgents, _1, &game_info));
    ros::Subscriber ghost_distance_subscriber = n.subscribe<pacman_interface::AgentPose>
                        ("/pacman_interface/ghost_distance", 1000, boost::bind(updateGhosts, _1, &game_info));
    ros::Subscriber pacman_pose_subscriber = n.subscribe<geometry_msgs::Pose>
                        ("/pacman_interface/pacman_pose", 1000, boost::bind(updatePacman, _1, &game_info));

    while (ros::ok())
    {
    //game_info.printPacmanOrGhostPose(true, 0);
        ros::spinOnce();
        loop_rate.sleep();
    }
}