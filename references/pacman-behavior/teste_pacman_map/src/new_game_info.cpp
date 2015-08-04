#include "teste_pacman_map/new_game_info.h"

#include <sstream>

#include "pacman_interface/PacmanAction.h"
#include "pacman_interface/AgentAction.h"
#include "pacman_interface/PacmanMapInfo.h"

int NewGameInfo::MAX_DISTANCE = 1000000;

NewGameInfo::NewGameInfo()
{
    ros::ServiceClient initInfoClient = n_.serviceClient<pacman_interface::PacmanMapInfo>("pacman_initialize_map_layout");
    pacman_interface::PacmanMapInfo initInfo;

    ros::service::waitForService("pacman_initialize_map_layout", -1);

    if (initInfoClient.call(initInfo))
    {
        std::vector<unsigned char> map_msg = initInfo.response.layout.map;
        int num_initialized_ghost = 0;

        width_ = initInfo.response.layout.width;
        height_ = initInfo.response.layout.height;

        num_ghosts_ = (int) initInfo.response.numGhosts;

        pacman_interface::MapLayout map_layout;

        std::vector<float> pose_line (width_, 0);
        pacman_pose_map_ = std::vector< std::vector<float> > (height_, pose_line);

        for (int i = 0 ; i < num_ghosts_; i++)
        {
            ghosts_poses_map_.push_back( std::vector< std::vector<float> > (height_, pose_line));
        }

        for (int i = 0 ; i < height_ ; i++) {
            std::vector<MapElements> map_line;
            std::vector<float> foods_map_line;
            for (int j = 0 ; j < width_ ; j++) {
                    float has_food = 0.0;

                    if (map_msg[i * width_ + j] == map_layout.EMPTY)
                        map_line.push_back(EMPTY);
                    else if (map_msg[i * width_ + j] == map_layout.FOOD)
                    {
                        map_line.push_back(EMPTY);
                        has_food = 1.0;
                    }
                    else if (map_msg[i * width_ + j] == map_layout.BIG_FOOD)
                        map_line.push_back(BIG_FOOD);
                    else if (map_msg[i * width_ + j] == map_layout.WALL)
                        map_line.push_back(WALL);
                    else if (map_msg[i * width_ + j] == map_layout.GHOST)
                    {
                        if (num_initialized_ghost < num_ghosts_)
                        {
                            geometry_msgs::Pose ghost_pose;
                            ghost_pose.position.x = j;
                            ghost_pose.position.y = i;
                            ghosts_poses_.push_back(ghost_pose);

                            geometry_msgs::Pose ghost_pose_sd;
                            ghost_pose.position.x = 0;
                            ghost_pose.position.y = 0;
                            ghosts_poses_sd_.push_back(ghost_pose_sd);

                            ghosts_poses_map_[num_initialized_ghost][i][j] = 1.0;

                            num_initialized_ghost++;
                        }
                        map_line.push_back(EMPTY);
                    }
                    else if (map_msg[i * width_ + j] == map_layout.PACMAN)
                    {
                        pacman_pose_.position.x = j;
                        pacman_pose_.position.y = i;
                        pacman_pose_sd_.position.x = 0;
                        pacman_pose_sd_.position.y = 0;

                        pacman_pose_map_[i][j] = 1.0;

                        map_line.push_back(EMPTY);
                    }
                    else
                    {
                        map_line.push_back(ERROR);
                        std::cout << "Error reading map";
                    }

                    foods_map_line.push_back(has_food);
            }
            map_.push_back(map_line);
            foods_map_.push_back(foods_map_line);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service PacmanInitializationInfo");
    }
}

void NewGameInfo::printMap()
{
    for (int i = height_ -1 ; i > -1  ; i--) {
        std::ostringstream foo;
        for (int j = 0 ; j < width_ ; j++) {
                bool is_ghost = false;
                for(int ghost_counter = 0; ghost_counter < num_ghosts_ ; ghost_counter++)
                {
                    if (ghosts_poses_[ghost_counter].position.x == j && ghosts_poses_[ghost_counter].position.y == i)
                    {
                        foo << 'G';
                        is_ghost = true;
                        break;
                    }
                }
                if (is_ghost)
                    continue;
                else if (pacman_pose_.position.x == j && pacman_pose_.position.y == i)
                    foo << 'P';
                else if (foods_map_[i][j] >= 0.3)
                {
                    if (foods_map_[i][j] >= 0.9)
                        foo << "\033[42m";
                    else if (foods_map_[i][j] >= 0.5)
                        foo << "\033[43m";
                    else if (foods_map_[i][j] >= 0.3)
                        foo << "\033[41m";
                    foo << '.' << "\033[0m";
                }
                else if (map_[i][j] == BIG_FOOD)
                    foo << 'B';
                else if (map_[i][j] == WALL)
                    foo << '#';
                else if(map_[i][j] == EMPTY)
                    foo << ' ';
                else
                    foo << 'E';
        }
        ROS_INFO_STREAM(foo.str());
    }
    ROS_INFO_STREAM("Sum: " << num_ghosts_ << std::endl);
}

void NewGameInfo::printMap2()
{
    for (int i = height_ -1 ; i > -1  ; i--) {
        std::ostringstream foo;
        for (int j = 0 ; j < width_ ; j++) {
                bool is_ghost = false;
                for(int ghost_counter = 0; ghost_counter < num_ghosts_ ; ghost_counter++)
                {
                    if (ghosts_poses_[ghost_counter].position.x == j && ghosts_poses_[ghost_counter].position.y == i)
                    {
                        foo << 'G';
                        is_ghost = true;
                        break;
                    }
                }
                if (is_ghost)
                    continue;
                else if (pacman_pose_.position.x == j && pacman_pose_.position.y == i)
                    foo << 'P';
                else if (foods_map_[i][j] >= 0.3)
                {
                    //foo << "\033[48;2;0;0;0m" << '.' << "\033[0m";
                    if (foods_map_[i][j] >= 0.9)
                        foo << "\033[48;5;46m";
                    else if (foods_map_[i][j] >= 0.5)
                        foo << "\033[48;5;30m";
                    else if (foods_map_[i][j] >= 0.3)
                        foo << "\033[48;5;22m";
                    foo << '.' << "\033[0m";
                }
                else if (map_[i][j] == BIG_FOOD)
                    foo << 'B';
                else if (map_[i][j] == WALL)
                    foo << '#';
                else if(map_[i][j] == EMPTY)
                    foo << ' ';
                else
                    foo << 'E';
        }
        ROS_INFO_STREAM(foo.str());
    }
    ROS_INFO_STREAM("Sum: " << num_ghosts_ << std::endl);
}

void NewGameInfo::printPacmanOrGhostPose( bool is_pacman, int ghost_index)
{
    for (int i = height_ -1 ; i > -1  ; i--) {
        std::ostringstream foo;
        foo << std::fixed;
        foo << std::setprecision(0);
        //foo << std::setw(5) << std::setfill('0');
        for (int j = 1 ; j < width_ - 1 ; j++) {
            if( getMapElement(j, i) == WALL)
                foo << "###" << ' ';
            else
            {
                int chance = -1;

                if (is_pacman)
                    chance = pacman_pose_map_[i][j]*100;
                else
                    chance = ghosts_poses_map_[ghost_index][i][j]*100;

                if (chance >= 90)
                    foo << "\033[48;5;46m";
                else if (chance >= 50)
                    foo << "\033[48;5;30m";
                else if (chance >= 30)
                    foo << "\033[48;5;22m";
                else
                    foo << "\033[48;5;12m";

                foo << std::setw(3) << std::setfill('0') << chance;

                foo << "\033[0m" << ' ';
            }
        }
        ROS_INFO_STREAM(foo.str());
    }
}

NewGameInfo::MapElements NewGameInfo::getMapElement(int x, int y)
{
    return map_[y][x];
}

int NewGameInfo::getHeight()
{
    return height_;
}

int NewGameInfo::getWidth()
{
    return width_;
}

std::vector< pacman_interface::PacmanAction > NewGameInfo::getLegalActions(int x, int y)
{
    std::vector< pacman_interface::PacmanAction > legal_actions;

    if(map_[y][x] == WALL)
    {
        return legal_actions;
    }

    if(map_[y + 1][x] != WALL)
    {
        pacman_interface::PacmanAction action;
        action.action = actions_.NORTH;
        legal_actions.push_back(action);
    }
    if(map_[y - 1][x] != WALL)
    {
        pacman_interface::PacmanAction action;
        action.action = actions_.SOUTH;
        legal_actions.push_back(action);
    }
    if(map_[y][x + 1] != WALL)
    {
        pacman_interface::PacmanAction action;
        action.action = actions_.EAST;
        legal_actions.push_back(action);
    }
    if(map_[y][x - 1] != WALL)
    {
        pacman_interface::PacmanAction action;
        action.action = actions_.WEST;
        legal_actions.push_back(action);
    }

    return legal_actions;
}

std::vector< std::pair<int, int> > NewGameInfo::getLegalNextPositions(int x, int y)
{
    std::vector< std::pair<int, int> > legal_next_positions;

    if(map_[y][x] == WALL)
    {
        return legal_next_positions;
    }

    if(map_[y + 1][x] != WALL)
    {
        legal_next_positions.push_back(std::make_pair(x, y+1));
    }
    if(map_[y - 1][x] != WALL)
    {
        legal_next_positions.push_back(std::make_pair(x, y-1));
    }
    if(map_[y][x + 1] != WALL)
    {
        legal_next_positions.push_back(std::make_pair(x+1, y));
    }
    if(map_[y][x - 1] != WALL)
    {
        legal_next_positions.push_back(std::make_pair(x-1, y));
    }

    return legal_next_positions;
}

std::vector< std::pair< float, std::pair<int, int> > > NewGameInfo::getNextPositionsForActionWithProbabilities(int x, int y, pacman_interface::PacmanAction action)
{
    std::vector< std::pair< float, std::pair<int, int> > > legal_next_positions_with_probabilities;

    if(map_[y][x] == WALL)
    {
        return legal_next_positions_with_probabilities;
    }

    float CHANCE_OF_ACTION_SUCCESS = 0.9;
    float chance_of_other_moves = (1 - CHANCE_OF_ACTION_SUCCESS )/4.0;

    float north_probability = ( action.action == actions_.NORTH ) ? CHANCE_OF_ACTION_SUCCESS : chance_of_other_moves;
    float south_probability = ( action.action == actions_.SOUTH ) ? CHANCE_OF_ACTION_SUCCESS : chance_of_other_moves;
    float west_probability = ( action.action == actions_.WEST ) ? CHANCE_OF_ACTION_SUCCESS : chance_of_other_moves;
    float east_probability = ( action.action == actions_.EAST ) ? CHANCE_OF_ACTION_SUCCESS : chance_of_other_moves;
    float stop_probability = ( action.action == actions_.STOP ) ? CHANCE_OF_ACTION_SUCCESS : chance_of_other_moves;

    if(map_[y + 1][x] != WALL)
        legal_next_positions_with_probabilities.push_back(std::make_pair(north_probability, std::make_pair(x, y+1)));
    else
        stop_probability += north_probability;
    if(map_[y - 1][x] != WALL)
        legal_next_positions_with_probabilities.push_back(std::make_pair(south_probability, std::make_pair(x, y-1)));
    else
        stop_probability += south_probability;
    if(map_[y][x + 1] != WALL)
        legal_next_positions_with_probabilities.push_back(std::make_pair(west_probability, std::make_pair(x+1, y)));
    else
        stop_probability += west_probability;
    if(map_[y][x - 1] != WALL)
        legal_next_positions_with_probabilities.push_back(std::make_pair(east_probability, std::make_pair(x-1, y)));
    else
        stop_probability += east_probability;
    legal_next_positions_with_probabilities.push_back(std::make_pair(stop_probability, std::make_pair(x, y)));

    return legal_next_positions_with_probabilities;
}

geometry_msgs::Pose NewGameInfo::getPacmanPose()
{
    return pacman_pose_;
}

std::vector< geometry_msgs::Pose > NewGameInfo::getGhostsPoses()
{
    return ghosts_poses_;
}

int NewGameInfo::getNumberOfGhosts()
{
    return num_ghosts_;
}

std::vector< std::vector<float> > NewGameInfo::getPacmanPoseMap()
{
    return pacman_pose_map_;
}

void NewGameInfo::setPacmanPoseMap(std::vector< std::vector<float> > pacman_pose_map)
{
    pacman_pose_map_ = pacman_pose_map;
}

std::vector< std::vector<float> > NewGameInfo::getGhostPoseMap(int ghost_index)
{
    return ghosts_poses_map_[ghost_index];
}

void NewGameInfo::setGhostPoseMap(std::vector< std::vector<float> > ghost_pose_map, int ghost_index)
{
    ghosts_poses_map_[ghost_index] = ghost_pose_map;
}