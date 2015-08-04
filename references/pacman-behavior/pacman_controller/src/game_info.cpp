#include "pacman_controller/game_info.h"

#include <sstream>

#include "util_functions.h"
#include "pacman_interface/PacmanAction.h"
#include "pacman_interface/AgentAction.h"
#include "pacman_interface/PacmanMapInfo.h"

int GameInfo::MAX_DISTANCE = 1000000;

GameInfo::GameInfo()
{
    update_world_subscriber_ = n_.subscribe<pacman_interface::AgentAction>("/pacman_interface/agent_action", 1000, boost::bind(&GameInfo::updateAgents, this, _1));

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

        for (int i = 0 ; i < height_ ; i++) {
            std::vector<MapElements> map_line;
            for (int j = 0 ; j < width_ ; j++) {
                    if (map_msg[i * width_ + j] == map_layout.EMPTY)
                        map_line.push_back(EMPTY);
                    else if (map_msg[i * width_ + j] == map_layout.FOOD)
                        map_line.push_back(FOOD);
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

                            num_initialized_ghost++;
                        }
                        map_line.push_back(EMPTY);
                    }
                    else if (map_msg[i * width_ + j] == map_layout.PACMAN)
                    {
                        pacman_pose_.position.x = j;
                        pacman_pose_.position.y = i;
                        map_line.push_back(EMPTY);
                    }
                    else
                    {
                        map_line.push_back(ERROR);
                        std::cout << "Error reading map";
                    }
            }
            map_.push_back(map_line);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service PacmanInitializationInfo");
    }
}

void GameInfo::printMap()
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
                else if (map_[i][j] == FOOD)
                    foo << '.';
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

GameInfo::MapElements GameInfo::getMapElement(int x, int y)
{
    return map_[y][x];
}

int GameInfo::getHeight()
{
    return height_;
}

int GameInfo::getWidth()
{
    return width_;
}

std::vector< pacman_interface::PacmanAction > GameInfo::getLegalActions(int x, int y)
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

std::vector< std::pair<int, int> > GameInfo::getLegalNextPositions(int x, int y)
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

std::map< std::pair<int, int>, int > GameInfo::calculateDistances(int x, int y)
{
    std::map< std::pair<int, int>, int > distances;

    distances[std::make_pair(x, y)] = 0;

    int current_distance = 0;
    bool done = false;

    std::vector< std::pair<int, int> > legal_positions = getLegalNextPositions(x, y);

    while(!done)
    {
        done = true;
        current_distance++;
        std::vector< std::pair<int, int> > next_legal_positions;

        for(std::vector< std::pair<int, int> >::reverse_iterator it = legal_positions.rbegin(); it != legal_positions.rend(); ++it)
        {
            if ( distances.find(*it) == distances.end() )
            {
                distances[*it] = current_distance;
                done = false;

                std::vector< std::pair<int, int> > temp_legal_positions = getLegalNextPositions(it->first, it->second);
                next_legal_positions.reserve(next_legal_positions.size() + temp_legal_positions.size());
                next_legal_positions.insert(next_legal_positions.end(), temp_legal_positions.begin(), temp_legal_positions.end());
            }
        }

        legal_positions = next_legal_positions;
    }
    
    return distances;
}

void GameInfo::precalculateAllDistances() {
    ROS_INFO_STREAM("Calculating all distances");

    for (int i = 0 ; i < width_ ; i++)
    {
        for (int j = 0 ; j < height_ ; j++)
        {
            if( map_[j][i] != WALL)
            {
                precalculated_distances_[std::make_pair(i, j)] = calculateDistances(i, j);
            }
        }
    }

    ROS_INFO_STREAM("Pre calculated all distances");
}

std::map< std::pair<int, int>, int > GameInfo::getDistances(int x, int y)
{
    std::map< std::pair<int, int>, int > distances = precalculated_distances_[std::make_pair(x, y)];
    
    return distances;
}

geometry_msgs::Pose GameInfo::getPacmanPose()
{
    return pacman_pose_;
}

std::vector< geometry_msgs::Pose > GameInfo::getGhostsPoses()
{
    return ghosts_poses_;
}

int GameInfo::getNumberOfGhosts()
{
    return num_ghosts_;
}

void GameInfo::updateAgents(const pacman_interface::AgentAction::ConstPtr& msg)
{
    geometry_msgs::Pose pose_diff = util::actionToMovement((int) msg->action);

    // ROS_INFO_STREAM("Agent is: " << (short) msg->agent << " with action " << (short) msg->action);

    if(msg->agent == msg->PACMAN)
    {
        pacman_pose_ = util::sumPoses(pacman_pose_, pose_diff);
        map_[pacman_pose_.position.y][pacman_pose_.position.x] = EMPTY;
    }
    else
    {
        int ghost_index = ( (short) msg->agent ) - 1;
        ghosts_poses_[ghost_index] = util::sumPoses(ghosts_poses_[ghost_index], pose_diff);
    }

    // printMap();
}